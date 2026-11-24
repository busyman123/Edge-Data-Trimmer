#include "edge_trimmer/mcap_writer.hpp"
#include <stdexcept>

namespace edge_trimmer {

NativeMcapWriter::NativeMcapWriter(const std::string& filepath)
    : out_(filepath, std::ios::binary) {
    if (!out_) {
        throw std::runtime_error("Failed to open MCAP output file: " + filepath);
    }
    const uint8_t magic[8] = {0x89, 0x4D, 0x43, 0x41, 0x50, 0x30, 0x0D, 0x0A};
    out_.write(reinterpret_cast<const char*>(magic), 8);

    std::vector<uint8_t> body;
    append_string(body, "waymo_trimmer");
    append_string(body, "native_cpp20");
    write_record(0x01, body);
}

uint16_t NativeMcapWriter::add_schema(const std::string& name, const std::string& encoding, const std::string& data) {
    const uint16_t id = next_schema_id_++;
    std::vector<uint8_t> body;
    append_u16(body, id);
    append_string(body, name);
    append_string(body, encoding);
    append_bytes(body, reinterpret_cast<const uint8_t*>(data.data()), data.size());
    write_record(0x03, body);
    return id;
}

uint16_t NativeMcapWriter::add_channel(uint16_t schema_id, const std::string& topic, const std::string& msg_encoding) {
    const uint16_t id = next_channel_id_++;
    std::vector<uint8_t> body;
    append_u16(body, id);
    append_u16(body, schema_id);
    append_string(body, topic);
    append_string(body, msg_encoding);
    append_u32(body, 0);
    write_record(0x04, body);
    return id;
}

void NativeMcapWriter::write_message(uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, const std::string& payload) {
    std::vector<uint8_t> body;
    append_u16(body, channel_id);
    append_u32(body, sequence);
    append_u64(body, log_time_ns);
    append_u64(body, log_time_ns);
    body.insert(body.end(), payload.begin(), payload.end());
    write_record(0x05, body);
}

void NativeMcapWriter::close() {
    if (!out_.is_open()) return;
    std::vector<uint8_t> data_end;
    append_u32(data_end, 0);
    write_record(0x0F, data_end);

    std::vector<uint8_t> footer;
    append_u64(footer, 0);
    append_u64(footer, 0);
    append_u32(footer, 0);
    write_record(0x02, footer);

    const uint8_t magic[8] = {0x89, 0x4D, 0x43, 0x41, 0x50, 0x30, 0x0D, 0x0A};
    out_.write(reinterpret_cast<const char*>(magic), 8);
    out_.close();
}

void NativeMcapWriter::write_record(uint8_t opcode, const std::vector<uint8_t>& body) {
    const uint64_t len = body.size();
    out_.put(static_cast<char>(opcode));
    out_.write(reinterpret_cast<const char*>(&len), sizeof(len));
    if (!body.empty()) {
        out_.write(reinterpret_cast<const char*>(body.data()), static_cast<std::streamsize>(body.size()));
    }
}

void NativeMcapWriter::append_u16(std::vector<uint8_t>& buf, uint16_t v) {
    const auto* p = reinterpret_cast<const uint8_t*>(&v);
    buf.insert(buf.end(), p, p + 2);
}

void NativeMcapWriter::append_u32(std::vector<uint8_t>& buf, uint32_t v) {
    const auto* p = reinterpret_cast<const uint8_t*>(&v);
    buf.insert(buf.end(), p, p + 4);
}

void NativeMcapWriter::append_u64(std::vector<uint8_t>& buf, uint64_t v) {
    const auto* p = reinterpret_cast<const uint8_t*>(&v);
    buf.insert(buf.end(), p, p + 8);
}

void NativeMcapWriter::append_string(std::vector<uint8_t>& buf, const std::string& s) {
    append_u32(buf, static_cast<uint32_t>(s.size()));
    buf.insert(buf.end(), s.begin(), s.end());
}

void NativeMcapWriter::append_bytes(std::vector<uint8_t>& buf, const uint8_t* data, size_t len) {
    append_u32(buf, static_cast<uint32_t>(len));
    buf.insert(buf.end(), data, data + len);
}

} // namespace edge_trimmer
