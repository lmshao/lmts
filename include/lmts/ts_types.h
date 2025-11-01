/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LMSHAO_LMTS_TS_TYPES_H
#define LMSHAO_LMTS_TS_TYPES_H

#include <lmcore/data_buffer.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace lmshao::lmts {

// TS packet size constants
constexpr size_t TS_PACKET_SIZE = 188;
constexpr size_t TS_HEADER_SIZE = 4;
constexpr size_t TS_PAYLOAD_SIZE = TS_PACKET_SIZE - TS_HEADER_SIZE;

// TS packet sync byte
constexpr uint8_t TS_SYNC_BYTE = 0x47;

// PID constants
constexpr uint16_t PAT_PID = 0x0000;
constexpr uint16_t PMT_PID = 0x1000;
constexpr uint16_t NULL_PID = 0x1FFF;

// Table ID constants
constexpr uint8_t PAT_TABLE_ID = 0x00;
constexpr uint8_t PMT_TABLE_ID = 0x02;

// PAT table structure constants
constexpr size_t PAT_TABLE_HEADER_SIZE = 8;
constexpr size_t PAT_PROGRAM_ENTRY_SIZE = 4;
constexpr size_t PAT_CRC_SIZE = 4;

// PMT table structure constants
constexpr size_t PMT_TABLE_HEADER_SIZE = 12;
constexpr size_t PMT_STREAM_ENTRY_SIZE = 5;
constexpr size_t PMT_CRC_SIZE = 4;

// Stream types
enum class StreamType : uint8_t {
    MPEG1_VIDEO = 0x01,
    MPEG2_VIDEO = 0x02,
    MPEG1_AUDIO = 0x03,
    MPEG2_AUDIO = 0x04,
    AAC_AUDIO = 0x0F,
    H264_VIDEO = 0x1B,
    H265_VIDEO = 0x24,
    AC3_AUDIO = 0x81,
    EAC3_AUDIO = 0x87
};

// TS packet header structure
struct TSHeader {
    uint8_t sync_byte; // Sync byte (0x47)
    uint8_t transport_error_indicator : 1;
    uint8_t payload_unit_start_indicator : 1;
    uint8_t transport_priority : 1;
    uint16_t pid : 13; // Packet identifier
    uint8_t transport_scrambling_control : 2;
    uint8_t adaptation_field_control : 2;
    uint8_t continuity_counter : 4;

    TSHeader()
        : sync_byte(TS_SYNC_BYTE), transport_error_indicator(0), payload_unit_start_indicator(0), transport_priority(0),
          pid(0), transport_scrambling_control(0), adaptation_field_control(0), continuity_counter(0)
    {
    }

    // Static method to parse header from raw data
    static TSHeader ParseFromData(const uint8_t *data);

    // Method to generate raw data from header
    void ToRawData(uint8_t *data) const;
};

// PAT entry structure
struct PATEntry {
    uint16_t program_number;
    uint16_t program_map_pid;

    PATEntry() : program_number(0), program_map_pid(0) {}
    PATEntry(uint16_t prog_num, uint16_t pmt_pid) : program_number(prog_num), program_map_pid(pmt_pid) {}
};

// PAT table structure
struct PATTable {
    uint16_t transport_stream_id;
    uint8_t version_number;
    bool current_next_indicator;
    uint8_t section_number;
    uint8_t last_section_number;
    std::vector<PATEntry> programs;

    PATTable()
        : transport_stream_id(0), version_number(0), current_next_indicator(false), section_number(0),
          last_section_number(0)
    {
    }

    // Parse PAT table from raw data
    static bool ParseFromData(const uint8_t *data, size_t size, PATTable &pat_table);
};

// PMT stream entry structure
struct PMTStreamEntry {
    uint8_t stream_type;
    uint16_t elementary_pid;
    uint16_t es_info_length;
    std::vector<uint8_t> es_info;

    PMTStreamEntry() : stream_type(0), elementary_pid(0), es_info_length(0) {}
};

// PMT table structure
struct PMTTable {
    uint16_t program_number;
    uint8_t version_number;
    bool current_next_indicator;
    uint8_t section_number;
    uint8_t last_section_number;
    uint16_t pcr_pid;
    uint16_t program_info_length;
    std::vector<uint8_t> program_info;
    std::vector<PMTStreamEntry> streams;

    PMTTable()
        : program_number(0), version_number(0), current_next_indicator(false), section_number(0),
          last_section_number(0), pcr_pid(0), program_info_length(0)
    {
    }

    // Parse PMT table from raw data
    static bool ParseFromData(const uint8_t *data, size_t size, PMTTable &pmt_table);
};

// TS packet structure
struct TSPacket {
    TSHeader header;
    std::shared_ptr<lmcore::DataBuffer> adaptation_field;
    std::shared_ptr<lmcore::DataBuffer> payload;

    TSPacket() = default;

    // Get total packet size
    size_t GetTotalSize() const;

    // Check if packet has PCR
    bool HasPCR() const;

    // Get PCR value (if present)
    uint64_t GetPCR() const;

    // Generate raw packet data
    std::vector<uint8_t> GetRawData() const;

    // Parse packet from raw data
    static bool ParseFromData(const uint8_t *data, size_t size, TSPacket &packet);
};

} // namespace lmshao::lmts

#endif // LMSHAO_LMTS_TS_TYPES_H