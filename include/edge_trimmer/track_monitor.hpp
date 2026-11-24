#pragma once

#include "edge_trimmer/types.hpp"

#include <string>
#include <unordered_set>
#include <vector>

namespace edge_trimmer {

/// Tracks 3D object IDs inside a safety radius and flags sudden perception dropouts
class TrackContinuityMonitor {
public:
    explicit TrackContinuityMonitor(double safety_radius_meters = 35.0)
        : safety_radius_m_(safety_radius_meters) {}

    std::vector<std::string> evaluate_frame(const std::vector<TrackedObject3D>& current_objects) {
        std::unordered_set<std::string> current_ids;
        current_ids.reserve(current_objects.size());

        for (const auto& obj : current_objects) {
            current_ids.insert(obj.object_id);
        }

        std::vector<std::string> dropped_ids;
        for (const auto& prev_id : prev_tracked_ids_in_zone_) {
            if (!current_ids.contains(prev_id)) {
                dropped_ids.push_back(prev_id);
            }
        }

        // Update tracked IDs that are currently inside the safety zone
        prev_tracked_ids_in_zone_.clear();
        for (const auto& obj : current_objects) {
            if (obj.distance_meters() <= safety_radius_m_) {
                prev_tracked_ids_in_zone_.insert(obj.object_id);
            }
        }

        return dropped_ids;
    }

private:
    double safety_radius_m_;
    std::unordered_set<std::string> prev_tracked_ids_in_zone_;
};

} // namespace edge_trimmer
