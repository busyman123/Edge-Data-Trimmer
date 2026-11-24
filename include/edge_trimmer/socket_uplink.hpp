#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace edge_trimmer {

/// Streams a serialized MCAP payload to the cloud ingestion listener (default 127.0.0.1:7447)
void stream_mcap_over_socket(const std::string& filepath,
                             const std::vector<uint8_t>& payload,
                             const std::string& host = "127.0.0.1",
                             uint16_t port = 7447);

} // namespace edge_trimmer
