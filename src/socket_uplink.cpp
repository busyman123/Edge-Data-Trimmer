#include "edge_trimmer/socket_uplink.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstddef>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

namespace edge_trimmer {

namespace {

bool send_all(int sock, const void* buf, size_t len) {
    const auto* ptr = static_cast<const uint8_t*>(buf);
    size_t sent_total = 0;
    while (sent_total < len) {
        ssize_t n = ::send(sock, ptr + sent_total, len - sent_total, 0);
        if (n <= 0) return false;
        sent_total += static_cast<size_t>(n);
    }
    return true;
}

} // namespace

void stream_mcap_over_socket(const std::string& filepath,
                             const std::vector<uint8_t>& payload,
                             const std::string& host,
                             uint16_t port) {
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    ::inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

    if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
        const std::string filename = fs::path(filepath).filename().string();
        const uint32_t name_len = htonl(static_cast<uint32_t>(filename.size()));
        const uint64_t data_len = payload.size();

        bool ok = send_all(sock, &name_len, sizeof(name_len)) &&
                  send_all(sock, filename.data(), filename.size()) &&
                  send_all(sock, &data_len, sizeof(data_len)) &&
                  send_all(sock, payload.data(), payload.size());

        if (ok) {
            std::cout << "[EDGE UPLINK] Streamed " << (payload.size() / 1024.0)
                      << " KB -> Cloud Ingestion Server (" << host << ":" << port << ")\n\n";
        }
    } else {
        std::cout << "[EDGE UPLINK] Cloud server offline; slice cached locally at " << filepath << "\n\n";
    }
    ::close(sock);
}

} // namespace edge_trimmer
