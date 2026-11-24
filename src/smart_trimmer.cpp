#include "edge_trimmer/smart_trimmer.hpp"
#include "edge_trimmer/foxglove_schemas.hpp"
#include "edge_trimmer/math_utils.hpp"
#include "edge_trimmer/mcap_writer.hpp"
#include "edge_trimmer/socket_uplink.hpp"

#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace edge_trimmer {

EdgeSmartTrimmer::EdgeSmartTrimmer(size_t max_flash_mb, size_t window_sec, size_t sample_hz)
    : flash_buffer_(max_flash_mb, window_sec, sample_hz),
      post_trigger_target_((window_sec / 2) * sample_hz) {}

void EdgeSmartTrimmer::process_frame(std::shared_ptr<JoinedSensorFrame> frame) {
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
    total_bytes_ingested_ += (frame->raw_multimodal_bytes > 0)
                                 ? frame->raw_multimodal_bytes
                                 : frame->size_bytes();

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

void EdgeSmartTrimmer::print_summary_report() const {
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

void EdgeSmartTrimmer::flush_event_to_uplink() {
    auto slice = flash_buffer_.snapshot();
    if (slice.empty()) {
        std::cout << "[TRIGGER] No data to flush." << std::endl;
        return;
    }

    fs::create_directories("./trimmed_events");
    const int64_t trigger_ts = slice.back()->timestamp_micros;
    const std::string output_path =
        "./trimmed_events/event_" + slice.back()->segment_id + "_" + std::to_string(trigger_ts) + ".mcap";

    NativeMcapWriter writer(output_path);

    const uint16_t telem_schema = writer.add_schema("waymo.Telemetry", "jsonschema", std::string(schemas::WAYMO_TELEMETRY));
    const uint16_t obj_schema   = writer.add_schema("waymo.TrackedObjects", "jsonschema", std::string(schemas::WAYMO_TRACKED_OBJECTS));
    const uint16_t scene_schema = writer.add_schema("foxglove.SceneUpdate", "jsonschema", std::string(schemas::FOXGLOVE_SCENE_UPDATE));
    const uint16_t img_schema   = writer.add_schema("foxglove.CompressedImage", "jsonschema", std::string(schemas::FOXGLOVE_COMPRESSED_IMAGE));
    const uint16_t log_schema   = writer.add_schema("foxglove.Log", "jsonschema", std::string(schemas::FOXGLOVE_LOG));

    const uint16_t log_ch   = writer.add_channel(log_schema, "/diagnostics/anomaly_log", "json");
    const uint16_t img_ch   = writer.add_channel(img_schema, "/camera/front/compressed", "json");
    const uint16_t telem_ch = writer.add_channel(telem_schema, "/vehicle/telemetry", "json");
    const uint16_t obj_ch   = writer.add_channel(obj_schema, "/perception/tracked_objects", "json");
    const uint16_t scene_ch = writer.add_channel(scene_schema, "/perception/scene_3d", "json");

    uint32_t seq = 0;
    int pulse_hold = 0;

    uint64_t prev_log_time_ns = 0;
    for (const auto& frame : slice) {
        if (frame->timestamp_micros <= 0) continue;

        uint64_t log_time_ns = static_cast<uint64_t>(frame->timestamp_micros) * 1000ULL;
        if (log_time_ns <= prev_log_time_ns) {
            log_time_ns = prev_log_time_ns + 100'000'000ULL; // +100ms (10Hz) fallback
        }
        prev_log_time_ns = log_time_ns;

        const uint32_t sec = static_cast<uint32_t>(log_time_ns / 1'000'000'000ULL);
        const uint32_t nsec = static_cast<uint32_t>(log_time_ns % 1'000'000'000ULL);

        if (frame->is_anomaly) {
            pulse_hold = 5; // Hold pulse = 1 for 0.5s (5 frames) so it's clearly visible in Plot
        }
        const int anomaly_triggered = (pulse_hold > 0) ? 1 : 0;
        if (pulse_hold > 0) --pulse_hold;

        // 1. Telemetry message
        std::ostringstream telem_json;
        telem_json << "{\"speed_mps\":" << safe_num(frame->ego_speed_mps)
                   << ",\"accel_x_mps2\":" << safe_num(frame->ego_accel_x_mps2)
                   << ",\"active_tracks\":" << frame->tracked_objects.size()
                   << ",\"dropped_count\":" << frame->dropped_track_ids.size()
                   << ",\"anomaly_triggered\":" << anomaly_triggered << "}";
        writer.write_message(telem_ch, seq++, log_time_ns, telem_json.str());

        // 2. TrackedObjects message
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

        // 3. Foxglove SceneUpdate 3D bounding boxes
        std::ostringstream scene_json;
        scene_json << "{\"deletions\":[],\"entities\":[";
        for (size_t i = 0; i < frame->tracked_objects.size(); ++i) {
            const auto& o = frame->tracked_objects[i];
            const double half_yaw = safe_num(o.heading) * 0.5;
            const double qz = std::sin(half_yaw);
            const double qw = std::cos(half_yaw);
            if (i > 0) 
                scene_json << ",";
            scene_json << "{\"timestamp\":{\"sec\":" << sec << ",\"nsec\":" << nsec << "}"
            << ",\"frame_id\":\"base_link\""
            << ",\"id\":\"" << o.object_id << "\""
            << ",\"lifetime\":{\"sec\":0,\"nsec\":150000000}"
            << ",\"frame_locked\":true"
            << ",\"cubes\":[{\"pose\":{\"position\":{\"x\":" << safe_num(o.center_x)
            << ",\"y\":" << safe_num(o.center_y)
            << ",\"z\":" << safe_num(o.center_z) << "}"
            << ",\"orientation\":{\"x\":0,\"y\":0,\"z\":" << qz << ",\"w\":" << qw << "}}"
            << ",\"size\":{\"x\":" << safe_num(o.length)
            << ",\"y\":" << safe_num(o.width)
            << ",\"z\":" << safe_num(o.height) << "}"
            << ",\"color\":{\"r\":0.1,\"g\":0.8,\"b\":0.4,\"a\":0.7}}]}";

        }
        scene_json << "]}";
        writer.write_message(scene_ch, seq++, log_time_ns, scene_json.str());
        // 4. Front camera CompressedImage (if available)
        if (!frame->front_camera_jpeg.empty()) {
            std::ostringstream img_json;
            img_json << "{\"timestamp\":{\"sec\":" << sec << ",\"nsec\":" << nsec << "}"
            << ",\"frame_id\":\"front_camera\""
            << ",\"data\":\"" << base64_encode(frame->front_camera_jpeg) << "\""
            << ",\"format\":\"jpeg\"}";
            writer.write_message(img_ch, seq++, log_time_ns, img_json.str());
        }
        // 5. Diagnostic log on anomaly frames
        if (frame->is_anomaly) {
            std::ostringstream log_json;
            log_json << "{\"timestamp\":{\"sec\":" << sec << ",\"nsec\":" << nsec << "}"
            << ",\"level\":2"
            << ",\"message\":\"" << frame->anomaly_reason << "\""
            << ",\"name\":\"edge_smart_trimmer\"}";
            writer.write_message(log_ch, seq++, log_time_ns, log_json.str());
        }
    }
    writer.close();
    // Update ROI counters and stream the generated .mcap slice to the cloud receiver
    ++total_events_triggered_;
    total_frames_exported_ += slice.size();
    std::ifstream in(output_path, std::ios::binary);
    std::vector<uint8_t> payload((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    total_mcap_bytes_written_ += payload.size();
    stream_mcap_over_socket(output_path, payload);
}
} // namespace edge_trimmer
