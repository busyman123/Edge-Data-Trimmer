#pragma once

#include "edge_trimmer/smart_trimmer.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

namespace edge_trimmer {

/// Collects and sorts all `.parquet` scene files in a directory
std::vector<std::filesystem::path> collect_parquet_scenes(const std::filesystem::path& dir);

/// Streams and joins vehicle_pose + lidar_box + camera_image + lidar Parquet tables in lockstep
void stream_joined_waymo_scene(const std::filesystem::path& pose_path,
                               const std::filesystem::path& lidar_box_path,
                               const std::filesystem::path& cam_path,
                               const std::filesystem::path& lidar_path,
                               EdgeSmartTrimmer& trimmer,
                               uint64_t sample_hz,
                               bool inject_fault);

} // namespace edge_trimmer
