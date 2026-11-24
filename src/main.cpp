#include "edge_trimmer/config.hpp"
#include "edge_trimmer/smart_trimmer.hpp"
#include "edge_trimmer/waymo_pipeline.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    bool inject_fault = false;
    fs::path config_path = fs::exists("./config/trimmer.conf")
                               ? "./config/trimmer.conf"
                               : "../config/trimmer.conf";


    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--inject-fault") {
            inject_fault = true;
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }

    if (inject_fault) {
        std::cout << "[MODE] Synthetic Fault Injection ENABLED (will trigger at frame 100)\n";
    }

    try {
        const auto cfg = edge_trimmer::load_config(config_path);
        std::cout << "[CONFIG] Loaded paths from " << config_path << "\n";

        constexpr uint64_t sample_hz = 10;
        edge_trimmer::EdgeSmartTrimmer trimmer(512, 6, sample_hz);

        auto pose_scenes = edge_trimmer::collect_parquet_scenes(cfg.pose_dir);
        if (pose_scenes.empty()) {
            std::cout << "No Parquet files found in " << cfg.pose_dir << ".\n"
                      << "Place matching segment .parquet files inside:\n"
                      << "  1) " << cfg.pose_dir << "\n"
                      << "  2) " << cfg.box_dir << "\n"
                      << "  3) " << cfg.cam_dir << "\n"
                      << "  4) " << cfg.lidar_dir << "\n";
            return 0;
        }

        std::cout << std::format("Found {} Waymo scenes. Starting multi-table 10Hz lockstep join...\n\n",
                                 pose_scenes.size());

        for (size_t idx = 0; idx < pose_scenes.size(); ++idx) {
            const auto& pose_path = pose_scenes[idx];
            const auto file_name  = pose_path.filename();
            const auto box_path   = cfg.box_dir / file_name;
            const auto cam_path   = cfg.cam_dir / file_name;
            const auto lidar_path = cfg.lidar_dir / file_name;

            if (!fs::exists(box_path)) {
                std::cout << "Skipping " << file_name << " (missing matching lidar_box file)\n";
                continue;
            }

            std::cout << std::format("Streaming Joined Scene [{}/{}]: {}\n",
                                     idx + 1, pose_scenes.size(), file_name.string());
            edge_trimmer::stream_joined_waymo_scene(pose_path, box_path, cam_path, lidar_path,
                                                    trimmer, sample_hz, inject_fault);
        }

        std::cout << "Completed streaming all joined Waymo scenes.\n";
        trimmer.print_summary_report();
    } catch (const std::exception& ex) {
        std::cerr << "Fatal error: " << ex.what() << '\n';
        return 1;
    }

    return 0;
}
