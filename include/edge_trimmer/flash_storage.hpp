#pragma once

#include "edge_trimmer/types.hpp"

#include <cstddef>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

namespace edge_trimmer {

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

} // namespace edge_trimmer
