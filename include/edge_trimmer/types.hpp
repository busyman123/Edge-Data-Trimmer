#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace edge_trimmer {

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
    std::string anomaly_reason = "NORMAL";
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

} // namespace edge_trimmer
