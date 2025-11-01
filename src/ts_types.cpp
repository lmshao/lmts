/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#include <lmcore/byte_order.h>
#include <lmts/ts_types.h>

#include <cstring>

#include "internal_logger.h"

namespace lmshao::lmts {

TSHeader TSHeader::ParseFromData(const uint8_t *data)
{
    TSHeader header;

    if (!data) {
        LMTS_LOGE("Invalid data pointer");
        return header;
    }

    header.sync_byte = data[0];
    header.transport_error_indicator = (data[1] >> 7) & 0x01;
    header.payload_unit_start_indicator = (data[1] >> 6) & 0x01;
    header.transport_priority = (data[1] >> 5) & 0x01;
    header.pid = ((data[1] & 0x1F) << 8) | data[2];
    header.transport_scrambling_control = (data[3] >> 6) & 0x03;
    header.adaptation_field_control = (data[3] >> 4) & 0x03;
    header.continuity_counter = data[3] & 0x0F;

    return header;
}

void TSHeader::ToRawData(uint8_t *data) const
{
    if (!data) {
        LMTS_LOGE("Invalid data pointer");
        return;
    }

    data[0] = sync_byte;
    data[1] = (transport_error_indicator << 7) | (payload_unit_start_indicator << 6) | (transport_priority << 5) |
              ((pid >> 8) & 0x1F);
    data[2] = pid & 0xFF;
    data[3] = (transport_scrambling_control << 6) | (adaptation_field_control << 4) | (continuity_counter & 0x0F);
}

bool PATTable::ParseFromData(const uint8_t *data, size_t size, PATTable &pat_table)
{
    if (!data || size < PAT_TABLE_HEADER_SIZE) {
        LMTS_LOGE("Invalid PAT data or size too small");
        return false;
    }

    // Check table ID
    if (data[0] != PAT_TABLE_ID) {
        LMTS_LOGE("Invalid PAT table ID: %d", data[0]);
        return false;
    }

    // Parse section length
    uint16_t section_length = ((data[1] & 0x0F) << 8) | data[2];
    if (section_length + 3 > size) {
        LMTS_LOGE("PAT section length exceeds available data");
        return false;
    }

    // Parse header fields
    pat_table.transport_stream_id = lmcore::ByteOrder::ReadBE16(&data[3]);
    pat_table.version_number = (data[5] >> 1) & 0x1F;
    pat_table.current_next_indicator = (data[5] & 0x01) != 0;
    pat_table.section_number = data[6];
    pat_table.last_section_number = data[7];

    // Parse program entries
    pat_table.programs.clear();
    size_t programs_length = section_length - 5 - PAT_CRC_SIZE; // Subtract header and CRC
    size_t offset = PAT_TABLE_HEADER_SIZE;

    while (offset + PAT_PROGRAM_ENTRY_SIZE <= size && programs_length >= PAT_PROGRAM_ENTRY_SIZE) {
        PATEntry entry;
        entry.program_number = lmcore::ByteOrder::ReadBE16(&data[offset]);
        entry.program_map_pid = lmcore::ByteOrder::ReadBE16(&data[offset + 2]) & 0x1FFF;

        pat_table.programs.push_back(entry);

        offset += PAT_PROGRAM_ENTRY_SIZE;
        programs_length -= PAT_PROGRAM_ENTRY_SIZE;
    }

    return true;
}

bool PMTTable::ParseFromData(const uint8_t *data, size_t size, PMTTable &pmt_table)
{
    if (!data || size < PMT_TABLE_HEADER_SIZE) {
        LMTS_LOGE("Invalid PMT data or size too small");
        return false;
    }

    // Check table ID
    if (data[0] != PMT_TABLE_ID) {
        LMTS_LOGE("Invalid PMT table ID: %d", data[0]);
        return false;
    }

    // Parse section length
    uint16_t section_length = ((data[1] & 0x0F) << 8) | data[2];
    if (section_length + 3 > size) {
        LMTS_LOGE("PMT section length exceeds available data");
        return false;
    }

    // Parse header fields
    pmt_table.program_number = lmcore::ByteOrder::ReadBE16(&data[3]);
    pmt_table.version_number = (data[5] >> 1) & 0x1F;
    pmt_table.current_next_indicator = (data[5] & 0x01) != 0;
    pmt_table.section_number = data[6];
    pmt_table.last_section_number = data[7];
    pmt_table.pcr_pid = lmcore::ByteOrder::ReadBE16(&data[8]) & 0x1FFF;
    pmt_table.program_info_length = lmcore::ByteOrder::ReadBE16(&data[10]) & 0x0FFF;

    // Parse program info
    size_t offset = PMT_TABLE_HEADER_SIZE;
    if (pmt_table.program_info_length > 0) {
        if (offset + pmt_table.program_info_length > size) {
            LMTS_LOGE("PMT program info length exceeds available data");
            return false;
        }
        pmt_table.program_info.assign(data + offset, data + offset + pmt_table.program_info_length);
        offset += pmt_table.program_info_length;
    }

    // Parse stream entries
    pmt_table.streams.clear();
    size_t remaining_length = section_length - (offset - 3) - PMT_CRC_SIZE;

    while (remaining_length >= PMT_STREAM_ENTRY_SIZE && offset + PMT_STREAM_ENTRY_SIZE <= size) {
        PMTStreamEntry entry;
        entry.stream_type = data[offset];
        entry.elementary_pid = lmcore::ByteOrder::ReadBE16(&data[offset + 1]) & 0x1FFF;
        entry.es_info_length = lmcore::ByteOrder::ReadBE16(&data[offset + 3]) & 0x0FFF;

        offset += PMT_STREAM_ENTRY_SIZE;
        remaining_length -= PMT_STREAM_ENTRY_SIZE;

        // Parse ES info
        if (entry.es_info_length > 0) {
            if (offset + entry.es_info_length > size || remaining_length < entry.es_info_length) {
                LMTS_LOGE("PMT ES info length exceeds available data");
                return false;
            }
            entry.es_info.assign(data + offset, data + offset + entry.es_info_length);
            offset += entry.es_info_length;
            remaining_length -= entry.es_info_length;
        }

        pmt_table.streams.push_back(entry);
    }

    return true;
}

size_t TSPacket::GetTotalSize() const
{
    return TS_PACKET_SIZE;
}

bool TSPacket::HasPCR() const
{
    return adaptation_field && adaptation_field->Size() >= 6 && (adaptation_field->Data()[0] & 0x10) != 0;
}

uint64_t TSPacket::GetPCR() const
{
    if (!HasPCR()) {
        return 0;
    }

    const uint8_t *pcr_data = adaptation_field->Data() + 1; // Skip adaptation field length
    uint64_t pcr_base = (static_cast<uint64_t>(pcr_data[0]) << 25) | (static_cast<uint64_t>(pcr_data[1]) << 17) |
                        (static_cast<uint64_t>(pcr_data[2]) << 9) | (static_cast<uint64_t>(pcr_data[3]) << 1) |
                        ((pcr_data[4] >> 7) & 0x01);

    uint16_t pcr_ext = ((pcr_data[4] & 0x01) << 8) | pcr_data[5];

    return pcr_base * 300 + pcr_ext;
}

std::vector<uint8_t> TSPacket::GetRawData() const
{
    std::vector<uint8_t> raw_data(TS_PACKET_SIZE,
                                  0xFF); // Fill with stuffing bytes

    // Write header
    header.ToRawData(raw_data.data());

    size_t offset = TS_HEADER_SIZE;

    // Write adaptation field if present
    if (adaptation_field && adaptation_field->Size() > 0) {
        size_t af_size = std::min(adaptation_field->Size(), TS_PACKET_SIZE - offset);
        std::memcpy(raw_data.data() + offset, adaptation_field->Data(), af_size);
        offset += af_size;
    }

    // Write payload if present
    if (payload && payload->Size() > 0 && offset < TS_PACKET_SIZE) {
        size_t payload_size = std::min(payload->Size(), TS_PACKET_SIZE - offset);
        std::memcpy(raw_data.data() + offset, payload->Data(), payload_size);
    }

    return raw_data;
}

bool TSPacket::ParseFromData(const uint8_t *data, size_t size, TSPacket &packet)
{
    if (!data || size < TS_PACKET_SIZE) {
        LMTS_LOGE("Invalid TS packet data or size");
        return false;
    }

    // Parse header
    packet.header = TSHeader::ParseFromData(data);

    if (packet.header.sync_byte != TS_SYNC_BYTE) {
        LMTS_LOGE("Invalid TS sync byte: 0x{:02X}", packet.header.sync_byte);
        return false;
    }

    size_t offset = TS_HEADER_SIZE;

    // Parse adaptation field
    if (packet.header.adaptation_field_control == 2 || packet.header.adaptation_field_control == 3) {
        if (offset >= TS_PACKET_SIZE) {
            LMTS_LOGE("No space for adaptation field");
            return false;
        }

        uint8_t af_length = data[offset];
        offset++;

        if (af_length > 0) {
            if (offset + af_length > TS_PACKET_SIZE) {
                LMTS_LOGE("Adaptation field length exceeds packet size");
                return false;
            }

            packet.adaptation_field = std::make_shared<lmcore::DataBuffer>(af_length + 1);
            packet.adaptation_field->Append(&data[offset - 1], af_length + 1);
            offset += af_length;
        }
    }

    // Parse payload
    if (packet.header.adaptation_field_control == 1 || packet.header.adaptation_field_control == 3) {
        if (offset < TS_PACKET_SIZE) {
            size_t payload_size = TS_PACKET_SIZE - offset;
            packet.payload = std::make_shared<lmcore::DataBuffer>(payload_size);
            packet.payload->Append(data + offset, payload_size);
        }
    }

    return true;
}

} // namespace lmshao::lmts