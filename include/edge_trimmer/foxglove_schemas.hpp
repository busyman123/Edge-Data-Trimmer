#pragma once

#include <string_view>

namespace edge_trimmer::schemas {

inline constexpr std::string_view WAYMO_TELEMETRY = R"({
    "type": "object",
    "properties": {
        "speed_mps": { "type": "number" },
        "accel_x_mps2": { "type": "number" },
        "active_tracks": { "type": "integer" },
        "dropped_count": { "type": "integer" },
        "anomaly_triggered": { "type": "integer" }
    }
})";

inline constexpr std::string_view WAYMO_TRACKED_OBJECTS = R"({
    "type": "object",
    "properties": {
        "dropped_ids": {
            "type": "array",
            "items": { "type": "string" }
        },
        "objects": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "id": { "type": "string" },
                    "x": { "type": "number" },
                    "y": { "type": "number" },
                    "vx": { "type": "number" },
                    "vy": { "type": "number" }
                }
            }
        }
    }
})";

inline constexpr std::string_view FOXGLOVE_SCENE_UPDATE = R"({
    "title": "foxglove.SceneUpdate",
    "type": "object",
    "properties": {
        "deletions": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                    "type": { "type": "integer" },
                    "id": { "type": "string" }
                }
            }
        },
        "entities": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                    "frame_id": { "type": "string" },
                    "id": { "type": "string" },
                    "lifetime": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
                    "frame_locked": { "type": "boolean" },
                    "metadata": { "type": "array", "items": { "type": "object", "properties": { "key": { "type": "string" }, "value": { "type": "string" } } } },
                    "arrows": { "type": "array", "items": { "type": "object" } },
                    "cubes": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "pose": {
                                    "type": "object",
                                    "properties": {
                                        "position": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" }, "z": { "type": "number" } } },
                                        "orientation": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" }, "z": { "type": "number" }, "w": { "type": "number" } } }
                                    }
                                },
                                "size": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" }, "z": { "type": "number" } } },
                                "color": { "type": "object", "properties": { "r": { "type": "number" }, "g": { "type": "number" }, "b": { "type": "number" }, "a": { "type": "number" } } }
                            }
                        }
                    },
                    "spheres": { "type": "array", "items": { "type": "object" } },
                    "cylinders": { "type": "array", "items": { "type": "object" } },
                    "lines": { "type": "array", "items": { "type": "object" } },
                    "triangles": { "type": "array", "items": { "type": "object" } },
                    "texts": { "type": "array", "items": { "type": "object" } },
                    "models": { "type": "array", "items": { "type": "object" } }
                }
            }
        }
    }
})";

inline constexpr std::string_view FOXGLOVE_COMPRESSED_IMAGE = R"({
    "type": "object",
    "properties": {
        "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
        "frame_id": { "type": "string" },
        "data": { "type": "string", "contentEncoding": "base64" },
        "format": { "type": "string" }
    }
})";

inline constexpr std::string_view FOXGLOVE_LOG = R"({
    "type": "object",
    "properties": {
        "timestamp": { "type": "object", "properties": { "sec": { "type": "integer" }, "nsec": { "type": "integer" } } },
        "level": { "type": "integer" },
        "message": { "type": "string" },
        "name": { "type": "string" }
    }
})";

} // namespace edge_trimmer::schemas
