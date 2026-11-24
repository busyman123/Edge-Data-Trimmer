#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace edge_trimmer {

/// Zero-dependency, spec-compliant MCAP v0 binary writer for Foxglove Studio
class NativeMcapWriter {
public:
    explicit NativeMcapWriter(const std::string& filepath);
    ~NativeMcapWriter() { close(); }

    uint16_t add_schema(const std::string& name, const std::string& encoding, const std::string& data);
    uint16_t add_channel(uint16_t schema_id, const std::string& topic, const std::string& msg_encoding);
    void write_message(uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, const std::string& payload);
    void close();

private:
    std::ofstream out_;
    uint16_t next_schema_id_ = 1;
    uint16_t next_channel_id_ = 1;

    void write_record(uint8_t opcode, const std::vector<uint8_t>& body);
    static void append_u16(std::vector<uint8_t>& buf, uint16_t v);
    static void append_u32(std::vector<uint8_t>& buf, uint32_t v);
    static void append_u64(std::vector<uint8_t>& buf, uint64_t v);
    static void append_string(std::vector<uint8_t>& buf, const std::string& s);
    static void append_bytes(std::vector<uint8_t>& buf, const uint8_t* data, size_t len);
};

} // namespace edge_trimmer
