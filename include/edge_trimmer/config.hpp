#pragma once

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace edge_trimmer {

struct PipelineConfig {
    std::filesystem::path pose_dir  = "./data/waymo_v2/vehicle_pose";
    std::filesystem::path box_dir   = "./data/waymo_v2/lidar_box";
    std::filesystem::path cam_dir   = "./data/waymo_v2/camera_image";
    std::filesystem::path lidar_dir = "./data/waymo_v2/lidar";
};

inline std::string trim_ws(const std::string& s) {
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

inline std::filesystem::path resolve_from_root(const std::filesystem::path& project_root,
                                               const std::filesystem::path& raw_path) {
    if (raw_path.is_absolute()) {
        return raw_path;
    }
    return std::filesystem::weakly_canonical(project_root / raw_path);
}

inline PipelineConfig load_config(const std::filesystem::path& config_path) {
    const auto abs_config = std::filesystem::weakly_canonical(config_path);
    std::ifstream file(abs_config);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open config file: " + abs_config.string());
    }

    // Assumes config lives in <project_root>/config/trimmer.conf
    const auto project_root = abs_config.parent_path().parent_path();

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    while (std::getline(file, line)) {
        if (const auto comment_pos = line.find('#'); comment_pos != std::string::npos) {
            line.erase(comment_pos);
        }
        line = trim_ws(line);
        if (line.empty()) continue;

        const auto eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string key = trim_ws(line.substr(0, eq_pos));
        std::string val = trim_ws(line.substr(eq_pos + 1));
        kv[key] = val;
    }

    PipelineConfig cfg;
    if (kv.contains("pose_dir"))  cfg.pose_dir  = kv["pose_dir"];
    if (kv.contains("box_dir"))   cfg.box_dir   = kv["box_dir"];
    if (kv.contains("cam_dir"))   cfg.cam_dir   = kv["cam_dir"];
    if (kv.contains("lidar_dir")) cfg.lidar_dir = kv["lidar_dir"];

    // Normalize relative paths against the project root
    cfg.pose_dir  = resolve_from_root(project_root, cfg.pose_dir);
    cfg.box_dir   = resolve_from_root(project_root, cfg.box_dir);
    cfg.cam_dir   = resolve_from_root(project_root, cfg.cam_dir);
    cfg.lidar_dir = resolve_from_root(project_root, cfg.lidar_dir);

    return cfg;
}

} // namespace edge_trimmer
