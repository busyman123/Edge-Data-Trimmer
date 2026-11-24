#include "edge_trimmer/parquet_cursor.hpp"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <stdexcept>
#include <string>

namespace edge_trimmer {

// ============================================================================
// LidarStreamCursor Implementation
// ============================================================================

LidarStreamCursor::LidarStreamCursor(const std::filesystem::path& parquet_path) {
    if (!std::filesystem::exists(parquet_path)) return;

    auto infile_res = arrow::io::ReadableFile::Open(parquet_path.string());
    if (!infile_res.ok()) {
        throw std::runtime_error("Failed to open lidar parquet file: " + infile_res.status().ToString());
    }

    parquet::ArrowReaderProperties props;
    props.set_batch_size(16); // Small batch size because LiDAR range images are large

    parquet::arrow::FileReaderBuilder builder;
    auto open_status = builder.Open(*infile_res);
    if (!open_status.ok()) {
        throw std::runtime_error("Failed to open lidar Parquet reader: " + open_status.ToString());
    }
    builder.properties(props);

    auto build_result = builder.Build();
    if (!build_result.ok()) {
        throw std::runtime_error("Failed to build lidar Parquet reader: " + build_result.status().ToString());
    }
    file_reader_ = std::move(*build_result);

    auto rb_reader_res = file_reader_->GetRecordBatchReader();
    if (!rb_reader_res.ok()) {
        throw std::runtime_error("Failed to get lidar RecordBatchReader: " + rb_reader_res.status().ToString());
    }
    batch_reader_ = std::move(*rb_reader_res);
    active_ = true;
}

std::pair<std::vector<uint8_t>, size_t> LidarStreamCursor::collect_for_timestamp(int64_t target_ts) {
    std::vector<uint8_t> top_lidar;
    size_t total_lidar_bytes = 0;
    if (!active_) return {top_lidar, 0};

    while (true) {
        while (peeked_.empty() && !eof_) refill();
        if (peeked_.empty()) break;

        auto& item = peeked_.front();
        if (item.ts < target_ts) {
            peeked_.pop_front();
        } else if (item.ts == target_ts) {
            total_lidar_bytes += item.raw_bytes.size();
            if (item.laser_name == 1) { // 1 = TOP LiDAR
                top_lidar = std::move(item.raw_bytes);
            }
            peeked_.pop_front();
        } else {
            break;
        }
    }
    return {std::move(top_lidar), total_lidar_bytes};
}

void LidarStreamCursor::refill() {
    std::shared_ptr<arrow::RecordBatch> batch;
    if (!batch_reader_->ReadNext(&batch).ok() || !batch) {
        eof_ = true;
        return;
    }

    auto ts_col = std::dynamic_pointer_cast<arrow::Int64Array>(
        batch->GetColumnByName("key.frame_timestamp_micros"));
    auto laser_id_col = batch->GetColumnByName("key.laser_name");
    auto range_col = std::dynamic_pointer_cast<arrow::BinaryArray>(
        batch->GetColumnByName("[LiDARComponent].range_image_return1.values"));

    for (int64_t r = 0; r < batch->num_rows(); ++r) {
        int64_t ts = ts_col ? ts_col->Value(r) : 0;

        int32_t laser_id = 0;
        if (auto i32 = std::dynamic_pointer_cast<arrow::Int32Array>(laser_id_col)) {
            laser_id = i32->Value(r);
        } else if (auto i8 = std::dynamic_pointer_cast<arrow::Int8Array>(laser_id_col)) {
            laser_id = static_cast<int32_t>(i8->Value(r));
        }

        std::vector<uint8_t> bytes;
        if (range_col && !range_col->IsNull(r)) {
            auto view = range_col->GetView(r);
            bytes.assign(reinterpret_cast<const uint8_t*>(view.data()),
                         reinterpret_cast<const uint8_t*>(view.data()) + view.size());
        }

        peeked_.push_back(LidarRow{ts, laser_id, std::move(bytes)});
    }
}

// ============================================================================
// LidarBoxStreamCursor Implementation
// ============================================================================

LidarBoxStreamCursor::LidarBoxStreamCursor(const std::filesystem::path& parquet_path) {
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

std::vector<TrackedObject3D> LidarBoxStreamCursor::collect_boxes_for_timestamp(int64_t target_ts) {
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

void LidarBoxStreamCursor::refill_queue() {
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

// ============================================================================
// CameraImageStreamCursor Implementation
// ============================================================================

CameraImageStreamCursor::CameraImageStreamCursor(const std::filesystem::path& parquet_path) {
    if (!std::filesystem::exists(parquet_path)) return;

    auto infile_res = arrow::io::ReadableFile::Open(parquet_path.string());
    if (!infile_res.ok()) {
        throw std::runtime_error("Failed to open camera image parquet file: " + infile_res.status().ToString());
    }

    parquet::ArrowReaderProperties props;
    props.set_batch_size(32); // Small batch size because JPEG frames are large

    parquet::arrow::FileReaderBuilder builder;
    auto open_status = builder.Open(*infile_res);
    if (!open_status.ok()) {
        throw std::runtime_error("Failed to open camera image Parquet reader: " + open_status.ToString());
    }
    builder.properties(props);

    auto build_result = builder.Build();
    if (!build_result.ok()) {
        throw std::runtime_error("Failed to build camera image Parquet reader: " + build_result.status().ToString());
    }
    file_reader_ = std::move(*build_result);

    auto rb_reader_res = file_reader_->GetRecordBatchReader();
    if (!rb_reader_res.ok()) {
        throw std::runtime_error("Failed to get camera image RecordBatchReader: " + rb_reader_res.status().ToString());
    }
    batch_reader_ = std::move(*rb_reader_res);
    active_ = true;
}

std::pair<std::vector<uint8_t>, size_t> CameraImageStreamCursor::collect_for_timestamp(int64_t target_ts) {
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

void CameraImageStreamCursor::refill() {
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
        int64_t ts = ts_col ? ts_col->Value(r) : 0;

        int32_t cam_id = 0;
        if (auto i32 = std::dynamic_pointer_cast<arrow::Int32Array>(cam_id_col)) {
            cam_id = i32->Value(r);
        } else if (auto i8 = std::dynamic_pointer_cast<arrow::Int8Array>(cam_id_col)) {
            cam_id = static_cast<int32_t>(i8->Value(r));
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

} // namespace edge_trimmer
