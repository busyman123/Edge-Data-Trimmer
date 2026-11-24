#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace edge_trimmer {

inline double safe_num(double v) noexcept {
    return std::isfinite(v) ? v : 0.0;
}

inline std::string base64_encode(const std::vector<uint8_t>& data) {
    static constexpr char tbl[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((data.size() + 2) / 3) * 4);
    for (size_t i = 0; i < data.size(); i += 3) {
        uint32_t v = static_cast<uint32_t>(data[i]) << 16;
        if (i + 1 < data.size()) v |= static_cast<uint32_t>(data[i + 1]) << 8;
        if (i + 2 < data.size()) v |= static_cast<uint32_t>(data[i + 2]);
        out.push_back(tbl[(v >> 18) & 0x3F]);
        out.push_back(tbl[(v >> 12) & 0x3F]);
        out.push_back((i + 1 < data.size()) ? tbl[(v >> 6) & 0x3F] : '=');
        out.push_back((i + 2 < data.size()) ? tbl[v & 0x3F] : '=');
    }
    return out;
}

} // namespace edge_trimmer
