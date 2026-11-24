#include "edge_trimmer/waymo_pipeline.hpp"
#include "edge_trimmer/parquet_cursor.hpp"
#include "edge_trimmer/track_monitor.hpp"
#include "edge_trimmer/types.hpp"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace fs = std::filesystem;

namespace edge_trimmer {

std::vector<fs::path> collect_parquet_scenes(const fs::path& dir) {
    std::vector<fs::path> scenes;
    if (fs::exists(dir) && fs::is_directory(dir)) {
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".parquet") {
                scenes.push_back(entry.path());
            }
        }
    }
    std::ranges::sort(scenes);
    return scenes;
}

void stream_joined_waymo_scene(const fs::path& pose_path,
                               const fs::path& lidar_box_path,
                               const fs::path& cam_path,
                               const fs::path& lidar_path,
                               EdgeSmartTrimmer& trimmer,
                               uint64_t sample_hz,
                               bool inject_fault) {
    auto infile_res = arrow::io::ReadableFile::Open(pose_path.string());
    if (!infile_res.ok()) {
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
    if (!reader_res.ok()) {
        throw std::runtime_error("Failed to build vehicle_pose reader: " + reader_res.status().ToString());
    }
    auto file_reader = std::move(*reader_res);

    auto rb_reader_res = file_reader->GetRecordBatchReader();
    if (!rb_reader_res.ok()) {
        throw std::runtime_error("Failed to get RecordBatchReader: " + rb_reader_res.status().ToString());
    }
    auto pose_reader = std::move(*rb_reader_res);

    LidarBoxStreamCursor box_cursor(lidar_box_path);
    CameraImageStreamCursor cam_cursor(cam_path);
    LidarStreamCursor lidar_cursor(lidar_path);
    TrackContinuityMonitor track_monitor(35.0);

    const auto frame_interval = std::chrono::milliseconds(1000 / sample_hz);
    std::optional<float> prev_speed;
    std::optional<int64_t> prev_ts;
    std::shared_ptr<arrow::RecordBatch> batch;
    size_t frame_counter = 0;

    while (true) {
        auto status = pose_reader->ReadNext(&batch);
        if (!status.ok()) {
            throw std::runtime_error("Error reading vehicle_pose batch: " + status.ToString());
        }
        if (!batch) break;

        auto segment_col = std::static_pointer_cast<arrow::StringArray>(
            batch->GetColumnByName("key.segment_context_name"));
        auto ts_col = std::static_pointer_cast<arrow::Int64Array>(
            batch->GetColumnByName("key.frame_timestamp_micros"));
        auto vel_x_col = std::static_pointer_cast<arrow::FloatArray>(
            batch->GetColumnByName("[VehiclePoseComponent].velocity.linear_velocity.x"));

        for (int64_t row = 0; row < batch->num_rows(); ++row) {
            ++frame_counter;
            std::string segment_id =
                segment_col ? segment_col->GetString(row) : "unknown_segment";
            if (!ts_col || ts_col->IsNull(row)) {
                continue;
            }
            int64_t timestamp_micros = ts_col->Value(row);
            if (timestamp_micros <= 0) {
                continue;
            }
            float ego_speed_mps = vel_x_col ? vel_x_col->Value(row) : 0.0f;
            float ego_accel_x_mps2 = 0.0f;

            if (prev_speed.has_value() && prev_ts.has_value() && timestamp_micros > *prev_ts) {
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
            if (inject_fault && frame_counter == 100) {
                ego_accel_x_mps2 = -6.8f;
                if (!tracked_objects.empty()) {
                    dropped_track_ids.push_back(tracked_objects.front().object_id);
                    tracked_objects.erase(tracked_objects.begin());
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

} // namespace edge_trimmer
