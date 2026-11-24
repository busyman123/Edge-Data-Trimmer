#pragma once

#include "edge_trimmer/types.hpp"

#include <arrow/record_batch.h>
#include <parquet/arrow/reader.h>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <utility>
#include <vector>

namespace edge_trimmer {

/// Streaming cursor that reads `lidar_box` Parquet batches in lockstep with `vehicle_pose`
class LidarBoxStreamCursor {
public:
    explicit LidarBoxStreamCursor(const std::filesystem::path& parquet_path);

    /// Collects all 3D bounding boxes matching `target_ts`, discarding stale earlier rows
    std::vector<TrackedObject3D> collect_boxes_for_timestamp(int64_t target_ts);

private:
    std::unique_ptr<parquet::arrow::FileReader> file_reader_;
    std::shared_ptr<arrow::RecordBatchReader> batch_reader_;
    std::deque<std::pair<int64_t, TrackedObject3D>> peeked_rows_;
    bool eof_ = false;

    void refill_queue();
};

/// Streams `camera_image` Parquet rows in lockstep, extracting FRONT camera (id == 1) JPEG bytes
class CameraImageStreamCursor {
public:
    explicit CameraImageStreamCursor(const std::filesystem::path& parquet_path);

    /// Returns {front_camera_jpeg_bytes, total_all_5_cameras_bytes_for_this_timestamp}
    std::pair<std::vector<uint8_t>, size_t> collect_for_timestamp(int64_t target_ts);

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

    void refill();
};

/// Streams `lidar` Parquet rows in lockstep, extracting TOP LiDAR (id == 1) range image bytes
class LidarStreamCursor {
public:
    explicit LidarStreamCursor(const std::filesystem::path& parquet_path);

    /// Returns {top_lidar_raw_bytes, total_all_5_lidars_bytes_for_this_timestamp}
    std::pair<std::vector<uint8_t>, size_t> collect_for_timestamp(int64_t target_ts);

private:
    struct LidarRow {
        int64_t ts;
        int32_t laser_name;
        std::vector<uint8_t> raw_bytes;
    };

    bool active_ = false;
    bool eof_ = false;
    std::unique_ptr<parquet::arrow::FileReader> file_reader_;
    std::shared_ptr<arrow::RecordBatchReader> batch_reader_;
    std::deque<LidarRow> peeked_;

    void refill();
};

} // namespace edge_trimmer

