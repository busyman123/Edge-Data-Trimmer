#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <format>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

inline double safe_num(double v) {
    return std::isfinite(v) ? v : 0.0;
}


/// Represents a single 3D detected object from Waymo's `lidar_box` table
struct TrackedObject3D {
    std::string object_id;
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double length = 4.5;
    double width = 1.9;
    double height = 1.6;
    double heading = 0.0; // Yaw in radians
    float speed_x = 0.0f;
    float speed_y = 0.0f;

    [[nodiscard]] double distance_meters() const noexcept {
        return std::hypot(center_x, center_y);
    }
};


/// Synchronized multi-table frame joining `vehicle_pose` + `lidar_box`
struct JoinedSensorFrame {
    std::string segment_id;
    int64_t timestamp_micros = 0;
    float ego_speed_mps = 0.0f;
    float ego_accel_x_mps2 = 0.0f;
    bool is_anomaly = false;
    std::string anomaly_reason = "NORMAL"; // Stores exact trigger cause
    std::vector<TrackedObject3D> tracked_objects;
    std::vector<std::string> dropped_track_ids;
    std::vector<uint8_t> front_camera_jpeg;
    std::vector<uint8_t> top_lidar_raw;
    size_t raw_multimodal_bytes = 0;

    [[nodiscard]] size_t size_bytes() const noexcept {
        return sizeof(*this) + segment_id.size() + anomaly_reason.size() +
               front_camera_jpeg.size() + top_lidar_raw.size() +
               (tracked_objects.size() * sizeof(TrackedObject3D));
    }
};


#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>



#include <fstream>
#include <sstream>

inline std::string base64_encode(const std::vector<uint8_t>& data) {
    static constexpr char tbl[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((data.size() + 2) / 3) * 4);
    for (size_t i = 0; i < data.size(); i += 3) {
        uint32_t v = static_cast<uint32_t>(data[i]) << 16;
        if (i + 1 < data.size()) v |= static_cast<uint32_t>(data[i + 1]) << 8;
        if (i + 2 < data.size()) v |= static_cast<uint32_t>(data[i + 2]);
        out.push_back(tbl[(v >> 18) & 0x3F]);
        out.push_back(tbl[(v >> 12) & 0x3F]);
        out.push_back((i + 1 < data.size()) ? tbl[(v >> 6) & 0x3F] : '=');
        out.push_back((i + 2 < data.size()) ? tbl[v & 0x3F] : '=');
    }
    return out;
}


/// Zero-dependency, spec-compliant MCAP v0 binary writer for Foxglove Studio
class NativeMcapWriter {
public:
    explicit NativeMcapWriter(const std::string& filepath)
        : out_(filepath, std::ios::binary) {
        if (!out_) {
            throw std::runtime_error("Failed to open MCAP output file: " + filepath);
        }
        // 1. Write MCAP v0 Magic Bytes (\x89MCAP0\r\n)
        const uint8_t magic[8] = {0x89, 0x4D, 0x43, 0x41, 0x50, 0x30, 0x0D, 0x0A};
        out_.write(reinterpret_cast<const char*>(magic), 8);

        // 2. Write Header Record (opcode 0x01): profile="waymo_trimmer", library="native_cpp20"
        std::vector<uint8_t> body;
        append_string(body, "waymo_trimmer");
        append_string(body, "native_cpp20");
        write_record(0x01, body);
    }

    uint16_t add_schema(const std::string& name, const std::string& encoding, const std::string& data) {
        const uint16_t id = next_schema_id_++;
        std::vector<uint8_t> body;
        append_u16(body, id);
        append_string(body, name);
        append_string(body, encoding);
        append_bytes(body, reinterpret_cast<const uint8_t*>(data.data()), data.size());
        write_record(0x03, body); // Schema opcode = 0x03
        return id;
    }

    uint16_t add_channel(uint16_t schema_id, const std::string& topic, const std::string& msg_encoding) {
        const uint16_t id = next_channel_id_++;
        std::vector<uint8_t> body;
        append_u16(body, id);
        append_u16(body, schema_id);
        append_string(body, topic);
        append_string(body, msg_encoding);
        append_u32(body, 0); // empty metadata map
        write_record(0x04, body); // Channel opcode = 0x04
        return id;
    }

    void write_message(uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, const std::string& payload) {
        std::vector<uint8_t> body;
        append_u16(body, channel_id);
        append_u32(body, sequence);
        append_u64(body, log_time_ns);
        append_u64(body, log_time_ns); // publish_time = log_time
        body.insert(body.end(), payload.begin(), payload.end());
        write_record(0x05, body); // Message opcode = 0x05
    }

    void close() {
        if (!out_.is_open()) return;
        // Write DataEnd (0x0F) + Footer (0x02) + Trailing Magic
        std::vector<uint8_t> data_end;
        append_u32(data_end, 0); // crc = 0 (disabled)
        write_record(0x0F, data_end);

        std::vector<uint8_t> footer;
        append_u64(footer, 0); // summary_start = 0 (unindexed stream)
        append_u64(footer, 0); // summary_offset_start = 0
        append_u32(footer, 0); // summary_crc = 0
        write_record(0x02, footer);

        const uint8_t magic[8] = {0x89, 0x4D, 0x43, 0x41, 0x50, 0x30, 0x0D, 0x0A};
        out_.write(reinterpret_cast<const char*>(magic), 8);
        out_.close();
    }

private:
    std::ofstream out_;
    uint16_t next_schema_id_ = 1;
    uint16_t next_channel_id_ = 1;

    void write_record(uint8_t opcode, const std::vector<uint8_t>& body) {
        const uint64_t len = body.size();
        out_.put(static_cast<char>(opcode));
        out_.write(reinterpret_cast<const char*>(&len), sizeof(len));
        if (!body.empty()) {
            out_.write(reinterpret_cast<const char*>(body.data()), static_cast<std::streamsize>(body.size()));
        }
    }

    static void append_u16(std::vector<uint8_t>& buf, uint16_t v) {
        const auto* p = reinterpret_cast<const uint8_t*>(&v);
        buf.insert(buf.end(), p, p + 2);
    }

    static void append_u32(std::vector<uint8_t>& buf, uint32_t v) {
        const auto* p = reinterpret_cast<const uint8_t*>(&v);
        buf.insert(buf.end(), p, p + 4);
    }

    static void append_u64(std::vector<uint8_t>& buf, uint64_t v) {
        const auto* p = reinterpret_cast<const uint8_t*>(&v);
        buf.insert(buf.end(), p, p + 8);
    }

    static void append_string(std::vector<uint8_t>& buf, const std::string& s) {
        append_u32(buf, static_cast<uint32_t>(s.size()));
        buf.insert(buf.end(), s.begin(), s.end());
    }

    static void append_bytes(std::vector<uint8_t>& buf, const uint8_t* data, size_t len) {
        append_u32(buf, static_cast<uint32_t>(len));
        buf.insert(buf.end(), data, data + len);
    }
};


/// Streaming cursor that reads `lidar_box` Parquet batches in lockstep with `vehicle_pose`
class LidarBoxStreamCursor {
public:
    explicit LidarBoxStreamCursor(const fs::path& parquet_path) {
        auto infile_res = arrow::io::ReadableFile::Open(parquet_path.string());
        if (!infile_res.ok()) {
            throw std::runtime_error("Failed to open lidar_box file: " + infile_res.status().ToString());
        }

        parquet::ArrowReaderProperties props;
        props.set_batch_size(256);

        parquet::arrow::FileReaderBuilder builder;
        auto open_status = builder.Open(*infile_res);
        if (!open_status.ok()) {
            throw std::runtime_error("Failed to open Parquet reader: " + open_status.ToString());
        }
        builder.properties(props);

        auto reader_res = builder.Build();
        
        if (!reader_res.ok()) {
            throw std::runtime_error("Failed to build Parquet reader: " + reader_res.status().ToString());
        }
        file_reader_ = std::move(*reader_res);

        auto rb_reader_res = file_reader_->GetRecordBatchReader();
        if (!rb_reader_res.ok()) {
            throw std::runtime_error("Failed to get RecordBatchReader: " + rb_reader_res.status().ToString());
        }
        batch_reader_ = std::move(*rb_reader_res);
    }

    /// Collects all 3D bounding boxes matching `target_ts`, discarding stale earlier rows
    std::vector<TrackedObject3D> collect_boxes_for_timestamp(int64_t target_ts) {
        std::vector<TrackedObject3D> matched;

        while (true) {
            while (peeked_rows_.empty() && !eof_) {
                refill_queue();
            }

            if (peeked_rows_.empty()) {
                break;
            }

            const auto& [ts, obj] = peeked_rows_.front();
            if (ts < target_ts) {
                peeked_rows_.pop_front();
            } else if (ts == target_ts) {
                matched.push_back(std::move(peeked_rows_.front().second));
                peeked_rows_.pop_front();
            } else {
                break;
            }
        }

        return matched;
    }

private:
    std::unique_ptr<parquet::arrow::FileReader> file_reader_;
    std::shared_ptr<arrow::RecordBatchReader> batch_reader_;
    std::deque<std::pair<int64_t, TrackedObject3D>> peeked_rows_;
    bool eof_ = false;

    void refill_queue() {
        if (eof_) return;

        std::shared_ptr<arrow::RecordBatch> batch;
        auto status = batch_reader_->ReadNext(&batch);
        if (!status.ok()) {
            throw std::runtime_error("Error reading lidar_box batch: " + status.ToString());
        }
        if (!batch) {
            eof_ = true;
            return;
        }

        auto ts_col = std::static_pointer_cast<arrow::Int64Array>(
            batch->GetColumnByName("key.frame_timestamp_micros"));
        auto id_col = std::static_pointer_cast<arrow::StringArray>(
            batch->GetColumnByName("key.laser_object_id"));
        auto cx_col = std::static_pointer_cast<arrow::DoubleArray>(
            batch->GetColumnByName("[LiDARBoxComponent].box.center.x"));
        auto cy_col = std::static_pointer_cast<arrow::DoubleArray>(
            batch->GetColumnByName("[LiDARBoxComponent].box.center.y"));
        auto vx_col = std::static_pointer_cast<arrow::FloatArray>(
            batch->GetColumnByName("[LiDARBoxComponent].speed.x"));
        auto vy_col = std::static_pointer_cast<arrow::FloatArray>(
            batch->GetColumnByName("[LiDARBoxComponent].speed.y"));
        auto cz_col = std::static_pointer_cast<arrow::DoubleArray>(
            batch->GetColumnByName("[LiDARBoxComponent].box.center.z"));
        auto len_col = std::static_pointer_cast<arrow::DoubleArray>(
            batch->GetColumnByName("[LiDARBoxComponent].box.size.x"));
        auto wid_col = std::static_pointer_cast<arrow::DoubleArray>(
            batch->GetColumnByName("[LiDARBoxComponent].box.size.y"));
        auto hgt_col = std::static_pointer_cast<arrow::DoubleArray>(
            batch->GetColumnByName("[LiDARBoxComponent].box.size.z"));
        auto hdg_col = std::static_pointer_cast<arrow::DoubleArray>(
            batch->GetColumnByName("[LiDARBoxComponent].box.heading"));


        for (int64_t row = 0; row < batch->num_rows(); ++row) {
            int64_t ts = ts_col ? ts_col->Value(row) : 0;
            TrackedObject3D obj{
                .object_id = id_col ? id_col->GetString(row) : ("obj_" + std::to_string(row)),
                .center_x = cx_col ? cx_col->Value(row) : 0.0,
                .center_y = cy_col ? cy_col->Value(row) : 0.0,
                .center_z = cz_col ? cz_col->Value(row) : 0.8,
                .length = len_col ? len_col->Value(row) : 4.5,
                .width = wid_col ? wid_col->Value(row) : 1.9,
                .height = hgt_col ? hgt_col->Value(row) : 1.6,
                .heading = hdg_col ? hdg_col->Value(row) : 0.0,
                .speed_x = vx_col ? vx_col->Value(row) : 0.0f,
                .speed_y = vy_col ? vy_col->Value(row) : 0.0f,
            };
            peeked_rows_.emplace_back(ts, std::move(obj));
        }
    }
};

/// Streams `camera_image` Parquet rows in lockstep, extracting FRONT camera (id == 1) JPEG bytes
class CameraImageStreamCursor {
public:
    explicit CameraImageStreamCursor(const fs::path& parquet_path) {
        if (!fs::exists(parquet_path)) return;
        auto infile = arrow::io::ReadableFile::Open(parquet_path.string()).ValueOrDie();
        parquet::ArrowReaderProperties props;
        props.set_batch_size(32); // Small batch size because JPEG frames are large

        parquet::arrow::FileReaderBuilder builder;
        auto open_status = builder.Open(infile);
        if (!open_status.ok()) {
            throw std::runtime_error("Failed to open camera image parquet file: " + open_status.ToString());
        }
        builder.properties(props);

        auto build_result = builder.Build();
        if (!build_result.ok()) {
            throw std::runtime_error("Failed to build camera image parquet reader: " + build_result.status().ToString());
        }
        file_reader_ = std::move(build_result.ValueOrDie());
        batch_reader_ = file_reader_->GetRecordBatchReader().ValueOrDie();
        active_ = true;
    }

    /// Returns {front_camera_jpeg_bytes, total_all_5_cameras_bytes_for_this_timestamp}
    std::pair<std::vector<uint8_t>, size_t> collect_for_timestamp(int64_t target_ts) {
        std::vector<uint8_t> front_jpeg;
        size_t total_cam_bytes = 0;
        if (!active_) return {front_jpeg, 0};

        while (true) {
            while (peeked_.empty() && !eof_) refill();
            if (peeked_.empty()) break;

            auto& item = peeked_.front();
            if (item.ts < target_ts) {
                peeked_.pop_front();
            } else if (item.ts == target_ts) {
                total_cam_bytes += item.jpeg_bytes.size();
                if (item.camera_name == 1) { // 1 = FRONT camera
                    front_jpeg = std::move(item.jpeg_bytes);
                }
                peeked_.pop_front();
            } else {
                break;
            }
        }
        return {std::move(front_jpeg), total_cam_bytes};
    }

private:
    struct CamRow {
        int64_t ts;
        int32_t camera_name;
        std::vector<uint8_t> jpeg_bytes;
    };

    bool active_ = false;
    bool eof_ = false;
    std::unique_ptr<parquet::arrow::FileReader> file_reader_;
    std::shared_ptr<arrow::RecordBatchReader> batch_reader_;
    std::deque<CamRow> peeked_;

    void refill() {
        std::shared_ptr<arrow::RecordBatch> batch;
        if (!batch_reader_->ReadNext(&batch).ok() || !batch) {
            eof_ = true;
            return;
        }

        auto ts_col = std::dynamic_pointer_cast<arrow::Int64Array>(
            batch->GetColumnByName("key.frame_timestamp_micros"));
        auto cam_id_col = batch->GetColumnByName("key.camera_name");
        auto img_col = std::dynamic_pointer_cast<arrow::BinaryArray>(
            batch->GetColumnByName("[CameraImageComponent].image"));

        for (int64_t r = 0; r < batch->num_rows(); ++r) {
            int64_t ts = (ts_col && !ts_col->IsNull(r)) ? ts_col->Value(r) : 0;
            int32_t cam_id = 0;
            if (auto i8 = std::dynamic_pointer_cast<arrow::Int8Array>(cam_id_col); i8 && !i8->IsNull(r)) {
                cam_id = i8->Value(r);
            } else if (auto i32 = std::dynamic_pointer_cast<arrow::Int32Array>(cam_id_col); i32 && !i32->IsNull(r)) {
                cam_id = i32->Value(r);
            }

            std::vector<uint8_t> bytes;
            if (img_col && !img_col->IsNull(r)) {
                auto view = img_col->GetView(r);
                bytes.assign(reinterpret_cast<const uint8_t*>(view.data()),
                             reinterpret_cast<const uint8_t*>(view.data()) + view.size());
            }
            peeked_.push_back(CamRow{ts, cam_id, std::move(bytes)});
        }
    }
};

/// Streams Waymo v2 `lidar` Parquet rows in lockstep, extracting TOP LiDAR (id == 1) range image bytes
class LidarStreamCursor {
public:
    explicit LidarStreamCursor(const fs::path& parquet_path) {
        if (!fs::exists(parquet_path)) return;
        auto infile = arrow::io::ReadableFile::Open(parquet_path.string()).ValueOrDie();
        parquet::ArrowReaderProperties props;
        props.set_batch_size(16); // Small batch size because LiDAR range images are large
        parquet::arrow::FileReaderBuilder builder;
        auto status = builder.Open(infile);
        if (!status.ok()) {
            throw std::runtime_error(status.ToString());
        }
        builder.properties(props);
        status = builder.Build(&file_reader_);
        if (!status.ok()) {
            throw std::runtime_error(status.ToString());
        }
        batch_reader_ = file_reader_->GetRecordBatchReader().ValueOrDie();
        active_ = true;
    }

    /// Returns {top_lidar_raw_bytes, total_all_5_lidars_bytes_for_this_timestamp}
    std::pair<std::vector<uint8_t>, size_t> collect_for_timestamp(int64_t target_ts) {
        std::vector<uint8_t> top_lidar_bytes;
        size_t total_lidar_bytes = 0;
        if (!active_) return {top_lidar_bytes, 0};

        while (true) {
            while (peeked_.empty() && !eof_) refill();
            if (peeked_.empty()) break;

            auto& item = peeked_.front();
            if (item.ts < target_ts) {
                peeked_.pop_front();
            } else if (item.ts == target_ts) {
                total_lidar_bytes += item.range_bytes.size();
                if (item.laser_name == 1) { // 1 = TOP 64-beam LiDAR
                    top_lidar_bytes = std::move(item.range_bytes);
                }
                peeked_.pop_front();
            } else {
                break;
            }
        }
        return {std::move(top_lidar_bytes), total_lidar_bytes};
    }

private:
    struct LidarRow {
        int64_t ts;
        int32_t laser_name;
        std::vector<uint8_t> range_bytes;
    };

    bool active_ = false;
    bool eof_ = false;
    std::unique_ptr<parquet::arrow::FileReader> file_reader_;
    std::shared_ptr<arrow::RecordBatchReader> batch_reader_;
    std::deque<LidarRow> peeked_;

    void refill() {
        std::shared_ptr<arrow::RecordBatch> batch;
        if (!batch_reader_->ReadNext(&batch).ok() || !batch) {
            eof_ = true;
            return;
        }

        auto ts_col = std::dynamic_pointer_cast<arrow::Int64Array>(
            batch->GetColumnByName("key.frame_timestamp_micros"));
        auto laser_id_col = batch->GetColumnByName("key.laser_name");

        // Waymo v2 stores compressed range image values in [LiDARComponent].range_image_return1.values
        auto range_col = batch->GetColumnByName("[LiDARComponent].range_image_return1.values");
        auto list_col = std::dynamic_pointer_cast<arrow::ListArray>(range_col);
        auto bin_col = std::dynamic_pointer_cast<arrow::BinaryArray>(range_col);

        for (int64_t r = 0; r < batch->num_rows(); ++r) {
            int64_t ts = (ts_col && !ts_col->IsNull(r)) ? ts_col->Value(r) : 0;

            int32_t laser_id = 0;
            if (auto i8 = std::dynamic_pointer_cast<arrow::Int8Array>(laser_id_col); i8 && !i8->IsNull(r)) {
                laser_id = i8->Value(r);
            } else if (auto i32 = std::dynamic_pointer_cast<arrow::Int32Array>(laser_id_col); i32 && !i32->IsNull(r)) {
                laser_id = i32->Value(r);
            }

            std::vector<uint8_t> bytes;
            if (bin_col && !bin_col->IsNull(r)) {
                auto view = bin_col->GetView(r);
                bytes.assign(reinterpret_cast<const uint8_t*>(view.data()),
                             reinterpret_cast<const uint8_t*>(view.data()) + view.size());
            } else if (list_col && !list_col->IsNull(r)) {
                // If stored as List<float>, copy the underlying float buffer as raw bytes
                auto slice = std::dynamic_pointer_cast<arrow::FloatArray>(list_col->value_slice(r));
                if (slice && slice->length() > 0) {
                    const uint8_t* raw_ptr = reinterpret_cast<const uint8_t*>(slice->raw_values());
                    const size_t byte_len = static_cast<size_t>(slice->length()) * sizeof(float);
                    bytes.assign(raw_ptr, raw_ptr + byte_len);
                }
            }

            peeked_.push_back(LidarRow{ts, laser_id, std::move(bytes)});
        }
    }
};


/// Detects sudden 3D perception tracking dropouts across consecutive frames
class TrackContinuityMonitor {
public:
    explicit TrackContinuityMonitor(double critical_radius_meters)
        : critical_radius_meters_(critical_radius_meters) {}

    std::vector<std::string> evaluate_frame(const std::vector<TrackedObject3D>& current_objects) {
        std::unordered_set<std::string> current_ids;
        current_ids.reserve(current_objects.size());
        for (const auto& obj : current_objects) {
            current_ids.insert(obj.object_id);
        }

        std::vector<std::string> sudden_dropouts;
        for (const auto& [prev_id, prev_obj] : previous_tracks_) {
            if (!current_ids.contains(prev_id) &&
                prev_obj.distance_meters() <= critical_radius_meters_) {
                sudden_dropouts.push_back(prev_id);
            }
        }

        previous_tracks_.clear();
        for (const auto& obj : current_objects) {
            previous_tracks_[obj.object_id] = obj;
        }

        return sudden_dropouts;
    }

private:
    std::unordered_map<std::string, TrackedObject3D> previous_tracks_;
    double critical_radius_meters_;
};

/// Byte-bounded NVMe flash ring buffer
class CircularFlashStorage {
public:
    CircularFlashStorage(size_t max_flash_mb, size_t window_sec, size_t sample_hz)
        : max_flash_bytes_(max_flash_mb * 1024 * 1024),
          max_frames_(window_sec * sample_hz) {}

    void write_frame(std::shared_ptr<JoinedSensorFrame> frame) {
        size_t frame_bytes = frame->size_bytes();

        while ((current_bytes_ + frame_bytes > max_flash_bytes_ || queue_.size() >= max_frames_) &&
               !queue_.empty()) {
            current_bytes_ -= queue_.front()->size_bytes();
            queue_.pop_front();
        }

        current_bytes_ += frame_bytes;
        queue_.push_back(std::move(frame));
    }

    [[nodiscard]] std::vector<std::shared_ptr<JoinedSensorFrame>> snapshot() const {
        return {queue_.begin(), queue_.end()};
    }

    [[nodiscard]] double utilization_mb() const noexcept {
        return static_cast<double>(current_bytes_) / (1024.0 * 1024.0);
    }

private:
    std::deque<std::shared_ptr<JoinedSensorFrame>> queue_;
    size_t current_bytes_ = 0;
    size_t max_flash_bytes_;
    size_t max_frames_;
};

inline bool send_all(int sock, const void* buf, size_t len) {
    const auto* ptr = static_cast<const uint8_t*>(buf);
    size_t sent_total = 0;
    while (sent_total < len) {
        ssize_t n = ::send(sock, ptr + sent_total, len - sent_total, 0);
        if (n <= 0) return false;
        sent_total += static_cast<size_t>(n);
    }
    return true;
}

inline void stream_mcap_over_socket(const std::string& filepath, const std::vector<uint8_t>& payload) {
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(7447);
    ::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

    if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
        const std::string filename = fs::path(filepath).filename().string();
        const uint32_t name_len = htonl(static_cast<uint32_t>(filename.size()));
        const uint64_t data_len = payload.size();

        bool ok = send_all(sock, &name_len, sizeof(name_len)) &&
                  send_all(sock, filename.data(), filename.size()) &&
                  send_all(sock, &data_len, sizeof(data_len)) &&
                  send_all(sock, payload.data(), payload.size());

        if (ok) {
            std::cout << "[EDGE UPLINK] Streamed " << (payload.size() / 1024.0)
                      << " KB -> Cloud Ingestion Server (127.0.0.1:7447)\n\n";
        }
    } else {
        std::cout << "[EDGE UPLINK] Cloud server offline; slice cached locally at " << filepath << "\n\n";
    }
    ::close(sock);
}

/// Orchestrates multi-modal anomaly triggers (Kinematics + 3D Track Continuity)
class EdgeSmartTrimmer 
{
    public:
        EdgeSmartTrimmer(size_t max_flash_mb, size_t window_sec, size_t sample_hz)
            : flash_buffer_(max_flash_mb, window_sec, sample_hz),
            post_trigger_target_((window_sec / 2) * sample_hz) {}

        void process_frame(std::shared_ptr<JoinedSensorFrame> frame) {
            const bool hard_brake = frame->ego_accel_x_mps2 < decel_threshold_;
            const bool track_dropout = !frame->dropped_track_ids.empty();

            frame->is_anomaly = (hard_brake || track_dropout);

            if (hard_brake && track_dropout) {
                frame->anomaly_reason = "CRITICAL: Phantom/Hard Brake (" +
                    std::to_string(frame->ego_accel_x_mps2) + " m/s^2) + 3D Track Dropout (" +
                    frame->dropped_track_ids.front() + ")";
            } else if (track_dropout) {
                frame->anomaly_reason = "PERCEPTION LOSS: 3D Track ID [" +
                    frame->dropped_track_ids.front() + "] vanished inside 35m safety zone";
            } else if (hard_brake) {
                frame->anomaly_reason = "KINEMATIC ANOMALY: Sudden Deceleration (" +
                    std::to_string(frame->ego_accel_x_mps2) + " m/s^2)";
            } else {
                frame->anomaly_reason = "NOMINAL DRIVING";
            }

            const int64_t ts = frame->timestamp_micros;
            const float accel = frame->ego_accel_x_mps2;
            const size_t active_count = frame->tracked_objects.size();
            const auto dropped = frame->dropped_track_ids;
            ++total_frames_ingested_;
            total_bytes_ingested_ += frame->size_bytes();
            flash_buffer_.write_frame(std::move(frame));

            if (!trigger_active_) {
                if (hard_brake || track_dropout) {
                    std::string reason;
                    if (hard_brake && track_dropout) {
                        reason = std::format("HARD BRAKE ({:.2f} m/s^2) + 3D TRACK DROPOUT ({} ids)",
                                            accel, dropped.size());
                    } else if (track_dropout) {
                        reason = std::format("3D PERCEPTION TRACK DROPOUT ({} ids vanished within 35m)",
                                            dropped.size());
                    } else {
                        reason = std::format("HARD DECELERATION ({:.2f} m/s^2)", accel);
                    }

                    std::cout << std::format(
                        "[TRIGGER] {} | ts: {} us | Active 3D Tracks: {} | Flash: {:.2f} MB\n",
                        reason, ts, active_count, flash_buffer_.utilization_mb());

                    trigger_active_ = true;
                    post_trigger_remaining_ = post_trigger_target_;
                }
            } else {
                if (post_trigger_remaining_ > 0) {
                    --post_trigger_remaining_;
                }
                if (post_trigger_remaining_ == 0) {
                    flush_event_to_uplink();
                    trigger_active_ = false;
                }
            }
        }
        void print_summary_report() const {
            const double ingested_mb = static_cast<double>(total_bytes_ingested_) / (1024.0 * 1024.0);
            const double exported_mb = static_cast<double>(total_mcap_bytes_written_) / (1024.0 * 1024.0);

            const size_t discarded_frames = (total_frames_ingested_ > total_frames_exported_)
                                                ? (total_frames_ingested_ - total_frames_exported_)
                                                : 0;

            const double frame_reduction_pct =
                (total_frames_ingested_ > 0)
                    ? (static_cast<double>(discarded_frames) / static_cast<double>(total_frames_ingested_)) * 100.0
                    : 0.0;

            const double byte_reduction_pct =
                (total_bytes_ingested_ > 0)
                    ? ((ingested_mb - exported_mb) / ingested_mb) * 100.0
                    : 0.0;

            // Project annual savings for a 100-vehicle test fleet generating 450 GB/vehicle/day at $0.18/GB
            constexpr double fleet_vehicles = 100.0;
            constexpr double daily_gb_per_car = 450.0;
            constexpr double cloud_cost_per_gb = 0.18;
            const double annual_baseline_usd = fleet_vehicles * daily_gb_per_car * 365.0 * cloud_cost_per_gb;
            const double projected_annual_savings_usd = annual_baseline_usd * (byte_reduction_pct / 100.0);

            std::cout << "\n====================================================================\n"
                    << "        EDGE SMART-TRIMMER — EXECUTIVE BENCHMARK SUMMARY            \n"
                    << "====================================================================\n"
                    << "  Anomalies Triggered        : " << total_events_triggered_ << " event(s)\n"
                    << "  Total Frames Streamed      : " << total_frames_ingested_ << " frames\n"
                    << "  Frames Saved (.mcap)       : " << total_frames_exported_ << " frames\n"
                    << "  Frames Discarded at Edge   : " << discarded_frames << " frames ("
                    << frame_reduction_pct << "% frame reduction)\n"
                    << "--------------------------------------------------------------------\n"
                    << "  Raw Stream Volume          : " << ingested_mb << " MB\n"
                    << "  Trimmed MCAP Upload Volume : " << exported_mb << " MB\n"
                    << "  Edge Bandwidth Reduction   : " << byte_reduction_pct << "% saved\n"
                    << "--------------------------------------------------------------------\n"
                    << "  Est. Annual Fleet Savings  : $" << static_cast<long long>(projected_annual_savings_usd)
                    << " / year (100-car fleet @ $0.18/GB)\n"
                    << "====================================================================\n";
        }

    private:
        CircularFlashStorage flash_buffer_;
        bool trigger_active_ = false;
        size_t post_trigger_remaining_ = 0;
        size_t post_trigger_target_;
        float decel_threshold_ = -4.5f;

         // Executive ROI & Data Reduction Counters
        size_t total_frames_ingested_ = 0;
        size_t total_bytes_ingested_ = 0;
        size_t total_frames_exported_ = 0;
        size_t total_mcap_bytes_written_ = 0;
        size_t total_events_triggered_ = 0;
        void flush_event_to_uplink() 
        {
            auto slice = flash_buffer_.snapshot();
            if (slice.empty()) 
            {
                std::cout << "[TRIGGER] No data to flush." << std::endl;
                return;
            }

            fs::create_directories("./trimmed_events");
            const int64_t trigger_ts = slice.back()->timestamp_micros;
            const std::string output_path =
                "./trimmed_events/event_" + slice.back()->segment_id + "_" + std::to_string(trigger_ts) + ".mcap";

            NativeMcapWriter writer(output_path);

            const uint16_t telem_schema = writer.add_schema(
                "waymo.Telemetry",
                "jsonschema",
                R"({
                "type": "object",
                "properties": {
                    "speed_mps": { "type": "number" },
                    "accel_x_mps2": { "type": "number" },
                    "active_tracks": { "type": "integer" },
                    "dropped_count": { "type": "integer" },
                    "anomaly_triggered": { "type": "integer" }
                }
                })"
            );

            const uint16_t obj_schema = writer.add_schema(
                "waymo.TrackedObjects",
                "jsonschema",
                R"({
                "type": "object",
                "properties": {
                    "dropped_ids": {
                    "type": "array",
                    "items": { "type": "string" }
                    },
                    "objects": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                        "id": { "type": "string" },
                        "x": { "type": "number" },
                        "y": { "type": "number" },
                        "vx": { "type": "number" },
                        "vy": { "type": "number" }
                        }
                    }
                    }
                }
                })"
            );

            const uint16_t scene_schema = writer.add_schema(
                "foxglove.SceneUpdate",
                "jsonschema",
                R"({
                "title": "foxglove.SceneUpdate",
                "type": "object",
                "properties": {
                    "deletions": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                        "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                        "type": { "type": "integer" },
                        "id": { "type": "string" }
                        }
                    }
                    },
                    "entities": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                        "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                        "frame_id": { "type": "string" },
                        "id": { "type": "string" },
                        "lifetime": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                        "frame_locked": { "type": "boolean" },
                        "metadata": { "type": "array", "items": { "type": "object", "properties": { "key": { "type": "string" }, "value": { "type": "string" } } } },
                        "arrows": { "type": "array", "items": { "type": "object" } },
                        "cubes": {
                            "type": "array",
                            "items": {
                            "type": "object",
                            "properties": {
                                "pose": {
                                "type": "object",
                                "properties": {
                                    "position": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" }, "z": { "type": "number" } } },
                                    "orientation": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" }, "z": { "type": "number" }, "w": { "type": "number" } } }
                                }
                                },
                                "size": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" }, "z": { "type": "number" } } },
                                "color": { "type": "object", "properties": { "r": { "type": "number" }, "g": { "type": "number" }, "b": { "type": "number" }, "a": { "type": "number" } } }
                            }
                            }
                        },
                        "spheres": { "type": "array", "items": { "type": "object" } },
                        "cylinders": { "type": "array", "items": { "type": "object" } },
                        "lines": { "type": "array", "items": { "type": "object" } },
                        "triangles": { "type": "array", "items": { "type": "object" } },
                        "texts": { "type": "array", "items": { "type": "object" } },
                        "models": { "type": "array", "items": { "type": "object" } }
                        }
                    }
                    }
                }
                })"
            );

            const uint16_t img_schema = writer.add_schema(
                "foxglove.CompressedImage",
                "jsonschema",
                R"({
                "type": "object",
                "properties": {
                    "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                    "frame_id": { "type": "string" },
                    "data": { "type": "string", "contentEncoding": "base64" },
                    "format": { "type": "string" }
                }
                })"
            );

            const uint16_t log_schema = writer.add_schema(
                "foxglove.Log",
                "jsonschema",
                R"({
                "type": "object",
                "properties": {
                    "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                    "level": { "type": "integer" },
                    "message": { "type": "string" },
                    "name": { "type": "string" }
                }
                })"
            );
            const uint16_t log_ch = writer.add_channel(log_schema, "/diagnostics/anomaly_log", "json");

            const uint16_t img_ch = writer.add_channel(img_schema, "/camera/front/compressed", "json");


            const uint16_t telem_ch = writer.add_channel(telem_schema, "/vehicle/telemetry", "json");
            const uint16_t obj_ch = writer.add_channel(obj_schema, "/perception/tracked_objects", "json");
            const uint16_t scene_ch = writer.add_channel(scene_schema, "/perception/scene_3d", "json");

            uint32_t seq = 0;
            int pulse_hold = 0;
            for (const auto& frame : slice) 
            {
                const uint64_t log_time_ns = static_cast<uint64_t>(frame->timestamp_micros) * 1000ULL;

                            //const int anomaly_triggered =
                //((frame->ego_accel_x_mps2 < decel_threshold_) || !frame->dropped_track_ids.empty()) ? 1 : 0;
                
                if (frame->is_anomaly) 
                {
                    pulse_hold = 5; // Hold pulse = 1 for 0.5s (5 frames) so it's clearly visible in Plot
                }
                const int anomaly_triggered = (pulse_hold > 0) ? 1 : 0;
                if (pulse_hold > 0) --pulse_hold;
                std::ostringstream telem_json;
                telem_json << "{\"speed_mps\":" << safe_num(frame->ego_speed_mps)
                        << ",\"accel_x_mps2\":" << safe_num(frame->ego_accel_x_mps2)
                        << ",\"active_tracks\":" << frame->tracked_objects.size()
                        << ",\"dropped_count\":" << frame->dropped_track_ids.size()
                        << ",\"anomaly_triggered\":" << anomaly_triggered << "}";
                writer.write_message(telem_ch, seq, log_time_ns, telem_json.str());

                std::ostringstream obj_json;
                obj_json << "{\"dropped_ids\":[";
                for (size_t i = 0; i < frame->dropped_track_ids.size(); ++i) {
                    if (i > 0) obj_json << ",";
                    obj_json << "\"" << frame->dropped_track_ids[i] << "\"";
                }
                obj_json << "],\"objects\":[";
                for (size_t i = 0; i < frame->tracked_objects.size(); ++i) {
                    const auto& o = frame->tracked_objects[i];
                    if (i > 0) obj_json << ",";
                    obj_json << "{\"id\":\"" << o.object_id << "\""
                            << ",\"x\":" << safe_num(o.center_x)
                            << ",\"y\":" << safe_num(o.center_y)
                            << ",\"vx\":" << safe_num(o.speed_x)
                            << ",\"vy\":" << safe_num(o.speed_y) << "}";
                }
                obj_json << "]}";
                writer.write_message(obj_ch, seq++, log_time_ns, obj_json.str());

                const uint64_t sec = log_time_ns / 1'000'000'000ULL;
                const uint32_t nsec = static_cast<uint32_t>(log_time_ns % 1'000'000'000ULL);

                std::ostringstream scene_json;
                scene_json << "{\"deletions\":[],\"entities\":[";

                // Entity 0: Ego Vehicle Box (Blue) at origin in base_link frame
                scene_json << "{"
                        << "\"timestamp\":{\"sec\":" << sec << ",\"nsec\":" << nsec << "},"
                        << "\"frame_id\":\"base_link\","
                        << "\"id\":\"ego_vehicle\","
                        << "\"lifetime\":{\"sec\":0,\"nsec\":150000000},"
                        << "\"frame_locked\":true,"
                        << "\"cubes\":[{"
                        << "\"pose\":{\"position\":{\"x\":0,\"y\":0,\"z\":0.8},"
                        << "\"orientation\":{\"x\":0,\"y\":0,\"z\":0,\"w\":1}},"
                        << "\"size\":{\"x\":4.8,\"y\":2.0,\"z\":1.6},"
                        << "\"color\":{\"r\":0.1,\"g\":0.5,\"b\":1.0,\"a\":0.85}"
                        << "}]}";

                // Entities 1..N: Tracked 3D LiDAR Bounding Boxes (Amber = Normal, Red = Trigger Frame)
                const bool is_alert_frame =
                    (frame->ego_accel_x_mps2 < decel_threshold_) || !frame->dropped_track_ids.empty();

                for (const auto& o : frame->tracked_objects) {
                    const double half_yaw = safe_num(o.heading) * 0.5;
                    const double qz = std::sin(half_yaw);
                    const double qw = std::cos(half_yaw);

                    // Highlight objects in red on anomaly frames, amber during normal tracking
                    const char* color_rgba = is_alert_frame
                        ? "{\"r\":1.0,\"g\":0.15,\"b\":0.15,\"a\":0.75}"
                        : "{\"r\":1.0,\"g\":0.70,\"b\":0.0,\"a\":0.65}";

                    scene_json << ",{"
                            << "\"timestamp\":{\"sec\":" << sec << ",\"nsec\":" << nsec << "},"
                            << "\"frame_id\":\"base_link\","
                            << "\"id\":\"" << o.object_id << "\","
                            << "\"lifetime\":{\"sec\":0,\"nsec\":150000000},"
                            << "\"frame_locked\":true,"
                            << "\"cubes\":[{"
                            << "\"pose\":{\"position\":{\"x\":" << safe_num(o.center_x)
                            << ",\"y\":" << safe_num(o.center_y)
                            << ",\"z\":" << safe_num(o.center_z) << "},"
                            << "\"orientation\":{\"x\":0,\"y\":0,\"z\":" << safe_num(qz)
                            << ",\"w\":" << safe_num(qw) << "}},"
                            << "\"size\":{\"x\":" << safe_num(o.length)
                            << ",\"y\":" << safe_num(o.width)
                            << ",\"z\":" << safe_num(o.height) << "},"
                            << "\"color\":" << color_rgba
                            << "}]}";
                }

                scene_json << "]}";
                writer.write_message(scene_ch, seq++, log_time_ns, scene_json.str());

                if (!frame->front_camera_jpeg.empty()) {
                    std::ostringstream img_json;
                    img_json << "{\"timestamp\":{\"sec\":" << sec << ",\"nsec\":" << nsec << "},"
                            << "\"frame_id\":\"front_camera\","
                            << "\"format\":\"jpeg\","
                            << "\"data\":\"" << base64_encode(frame->front_camera_jpeg) << "\"}";
                    writer.write_message(img_ch, seq++, log_time_ns, img_json.str());
                }

                if (frame->is_anomaly || (seq % 10 == 0)) {
                    const int log_level = frame->is_anomaly ? 4 : 2; // 4 = ERROR (Red), 2 = INFO
                    std::ostringstream log_json;
                    log_json << "{\"timestamp\":{\"sec\":" << sec << ",\"nsec\":" << nsec << "},"
                            << "\"level\":" << log_level << ","
                            << "\"name\":\"EdgeSmartTrimmer\","
                            << "\"message\":\"" << frame->anomaly_reason << "\"}";
                    writer.write_message(log_ch, seq++, log_time_ns, log_json.str());
                }
            }

            writer.close();

            const size_t mcap_bytes = fs::file_size(output_path);
            ++total_events_triggered_;
            total_frames_exported_ += slice.size();
            total_mcap_bytes_written_ += mcap_bytes;

            const double file_kb = static_cast<double>(mcap_bytes) / 1024.0;
            std::cout << "[MCAP EXPORT] Wrote " << slice.size() << " synced frames -> "
                    << output_path << " (" << file_kb << " KB)\n\n";
            
            // Read finalized .mcap bytes and push over edge-to-cloud socket
            std::ifstream mcap_in(output_path, std::ios::binary);
            std::vector<uint8_t> mcap_bytes_(
                (std::istreambuf_iterator<char>(mcap_in)),
                std::istreambuf_iterator<char>());

            stream_mcap_over_socket(output_path, mcap_bytes_);
        }

};
/// Streams and joins vehicle_pose + lidar_box Parquet tables in lockstep
void stream_joined_waymo_scene(const fs::path& pose_path,   const fs::path& lidar_box_path,
                        const fs::path& cam_path, const fs::path& lidar_path,
                        EdgeSmartTrimmer& trimmer,  uint64_t sample_hz, bool inject_fault) 
{
    auto infile_res = arrow::io::ReadableFile::Open(pose_path.string());
    if (!infile_res.ok()) 
    {
        throw std::runtime_error("Failed to open vehicle_pose file: " + infile_res.status().ToString());
    }
    parquet::ArrowReaderProperties props;
    props.set_batch_size(32);

    parquet::arrow::FileReaderBuilder builder;
    auto open_status = builder.Open(*infile_res);
    if (!open_status.ok()) {
        throw std::runtime_error("Failed to open Parquet reader: " + open_status.ToString());
    }
    builder.properties(props);

    auto reader_res = builder.Build();
    if (!reader_res.ok()) 
    {
        throw std::runtime_error("Failed to build vehicle_pose reader: " + reader_res.status().ToString());
    }
    auto file_reader = std::move(*reader_res);

    auto rb_reader_res = file_reader->GetRecordBatchReader();
    if (!rb_reader_res.ok()) 
    {
        throw std::runtime_error("Failed to get RecordBatchReader: " + rb_reader_res.status().ToString());
    }
    auto pose_reader = std::move(*rb_reader_res);
    LidarBoxStreamCursor box_cursor(lidar_box_path);
    CameraImageStreamCursor cam_cursor(cam_path);
    LidarStreamCursor lidar_cursor(lidar_path);
    TrackContinuityMonitor track_monitor(35.0);
    const auto frame_interval = std::chrono::milliseconds(1000 / sample_hz);
    std::optional<int64_t> prev_speed;
    std::optional<int64_t> prev_ts;
    std::shared_ptr<arrow::RecordBatch> batch;
    while (true) 
    {
        auto status = pose_reader->ReadNext(&batch);
        if (!status.ok()) 
        {
            throw std::runtime_error("Error reading vehicle_pose batch: " + status.ToString());
        }
        if (!batch) break;
        auto segment_col = std::static_pointer_cast<arrow::StringArray>(
            batch->GetColumnByName("key.segment_context_name"));

        auto ts_col = std::static_pointer_cast<arrow::Int64Array>(
            batch->GetColumnByName("key.frame_timestamp_micros"));
        auto vel_x_col = std::static_pointer_cast<arrow::FloatArray>(
            batch->GetColumnByName("[VehiclePoseComponent].velocity.linear_velocity.x"));
        
        size_t frame_counter = 0;
        for (int64_t row = 0; row < batch->num_rows(); ++row) 
        {
            ++frame_counter;
            std::string segment_id =
                segment_col ? segment_col->GetString(row) : "unknown_segment";
            int64_t timestamp_micros = ts_col ? ts_col->Value(row) : 0;
            float ego_speed_mps = vel_x_col ? vel_x_col->Value(row) : 0.0f;
            float ego_accel_x_mps2 = 0.0f;

            if (prev_speed.has_value() && prev_ts.has_value() && timestamp_micros > *prev_ts) 
            {
                float dt_sec = static_cast<float>(timestamp_micros - *prev_ts) / 1'000'000.0f;
                ego_accel_x_mps2 = (ego_speed_mps - *prev_speed) / dt_sec;
            }
            prev_speed = ego_speed_mps;
            prev_ts = timestamp_micros;
            auto tracked_objects = box_cursor.collect_boxes_for_timestamp(timestamp_micros);
            auto [front_jpeg, total_cam_bytes] = cam_cursor.collect_for_timestamp(timestamp_micros);
            auto [top_lidar, total_lidar_bytes] = lidar_cursor.collect_for_timestamp(timestamp_micros);
            auto dropped_track_ids = track_monitor.evaluate_frame(tracked_objects);
            // Deterministically inject a phantom brake + 3D track dropout at frame 100
            if (inject_fault && frame_counter == 100) 
            {
                ego_accel_x_mps2 = -6.8f; // Simulates sudden phantom emergency brake
                if (!tracked_objects.empty()) {
                    dropped_track_ids.push_back(tracked_objects.front().object_id);
                    tracked_objects.erase(tracked_objects.begin()); // Remove closest object from frame
                }
            }
            auto frame = std::make_shared<JoinedSensorFrame>(JoinedSensorFrame{
                .segment_id = std::move(segment_id),
                .timestamp_micros = timestamp_micros,
                .ego_speed_mps = ego_speed_mps,
                .ego_accel_x_mps2 = ego_accel_x_mps2,
                .tracked_objects = std::move(tracked_objects),
                .dropped_track_ids = std::move(dropped_track_ids),
                .front_camera_jpeg = std::move(front_jpeg),
                .top_lidar_raw = std::move(top_lidar),
                .raw_multimodal_bytes = total_cam_bytes + total_lidar_bytes,
            });
            trimmer.process_frame(std::move(frame));
            std::this_thread::sleep_for(frame_interval);
        }
    }
}
std::vector<fs::path> collect_parquet_scenes(const fs::path& dir) 
{
    std::vector<fs::path> scenes;
    if (fs::exists(dir) && fs::is_directory(dir)) 
    {
        for (const auto& entry : fs::directory_iterator(dir)) 
        {
            if (entry.is_regular_file() && entry.path().extension() == ".parquet") 
            {
                scenes.push_back(entry.path());
            }
        }
    }
    std::ranges::sort(scenes);
    return scenes;
}

int main(int argc, char* argv[]) 
{
    bool inject_fault = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--inject-fault") {
            inject_fault = true;
        }
    }
    if (inject_fault) {
        std::cout << "[MODE] Synthetic Fault Injection ENABLED (will trigger at frame 100)\n";
    }

    try 
    {
        const fs::path pose_dir = "./data/waymo_v2/vehicle_pose";
        const fs::path box_dir = "./data/waymo_v2/lidar_box";
        const fs::path cam_dir  = "./data/waymo_v2/camera_image";
        const fs::path lidar_dir = "./data/waymo_v2/lidar";
        constexpr uint64_t sample_hz = 10;
        EdgeSmartTrimmer trimmer(512, 6, sample_hz);
        auto pose_scenes = collect_parquet_scenes(pose_dir);
        if (pose_scenes.empty()) 
        {
            std::cout << "No Parquet files found in " << pose_dir << ".\n"
                << "Place matching segment .parquet files inside both:\n"
                << "  1) ./data/waymo_v2/vehicle_pose/\n"
                << "  2) ./data/waymo_v2/lidar_box/\n"
                << "  3) ./data/waymo_v2/camera_image/\n"
                << "  4) ./data/waymo_v2/lidar/\n";
            return 0;
        }
        std::cout << std::format("Found {} Waymo scenes. Starting multi-table 10Hz lockstep join...\n\n",
        pose_scenes.size());
        for (size_t idx = 0; idx < pose_scenes.size(); ++idx) 
        {
            const auto& pose_path = pose_scenes[idx];
            const auto file_name = pose_path.filename();
            const auto box_path = box_dir / file_name;
            const fs::path cam_path = cam_dir / file_name;
            const fs::path lidar_path = lidar_dir / file_name;

            if (!fs::exists(box_path)) 
            {
                std::cout << "Skipping " << file_name << " (missing matching lidar_box file)\n";
                continue;
            }
            std::cout << std::format("Streaming Joined Scene [{}/{}]: {}\n",
                idx + 1, pose_scenes.size(), file_name.string());
            stream_joined_waymo_scene(pose_path, box_path, cam_path, lidar_path,
                                      trimmer, sample_hz, inject_fault);
        }
        std::cout << "Completed streaming all joined Waymo scenes.\n";
        trimmer.print_summary_report();
    } 
    catch (const std::exception& ex) 
    {
        std::cerr << "Fatal error: " << ex.what() << '\n';
        return 1;
    }
    return 0;
}
