#pragma once

#include "edge_trimmer/flash_storage.hpp"
#include "edge_trimmer/types.hpp"

#include <cstddef>
#include <memory>

namespace edge_trimmer {

/// Orchestrates multi-modal anomaly triggers (Kinematics + 3D Track Continuity)
class EdgeSmartTrimmer {
public:
    EdgeSmartTrimmer(size_t max_flash_mb, size_t window_sec, size_t sample_hz);

    void process_frame(std::shared_ptr<JoinedSensorFrame> frame);
    void print_summary_report() const;

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

    void flush_event_to_uplink();
};

} // namespace edge_trimmer
