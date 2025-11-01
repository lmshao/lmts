/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#include <lmcore/byte_order.h>
#include <lmts/ts_demuxer.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>
#include <set>

#include "internal_logger.h"

namespace lmshao::lmts {

class TSDemuxerImpl {
public:
    TSDemuxerImpl() : running_(false) {}

    ~TSDemuxerImpl() { Stop(); }

    void SetStreamDataCallback(StreamDataCallback callback) { stream_data_callback_ = std::move(callback); }

    void SetStreamInfoCallback(StreamInfoCallback callback) { stream_info_callback_ = std::move(callback); }

    void SetPATCallback(PATCallback callback) { pat_callback_ = std::move(callback); }

    void SetPMTCallback(PMTCallback callback) { pmt_callback_ = std::move(callback); }

    bool Start()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (running_) {
            LMTS_LOGW("Demuxer is already running");
            return true;
        }

        running_ = true;
        Reset();

        LMTS_LOGI("TS Demuxer started");
        return true;
    }

    void Stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        LMTS_LOGI("TS Demuxer stopped");
    }

    bool IsRunning() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return running_;
    }

    size_t ParseData(const uint8_t *data, size_t size)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!running_) {
            LMTS_LOGE("Demuxer is not running");
            return 0;
        }

        size_t processed = 0;
        const uint8_t *current = data;

        while (processed + TS_PACKET_SIZE <= size) {
            // Find sync byte
            if (*current != TS_SYNC_BYTE) {
                // Search for next sync byte
                const uint8_t *sync_pos = std::find(current, data + size, TS_SYNC_BYTE);
                if (sync_pos == data + size) {
                    break; // No more sync bytes found
                }
                size_t skipped = sync_pos - current;
                current = sync_pos;
                processed += skipped;
                statistics_["sync_errors"]++;
                continue;
            }

            // Parse TS packet
            TSPacket packet;
            if (TSPacket::ParseFromData(current, TS_PACKET_SIZE, packet)) {
                ProcessTSPacket(packet);
                statistics_["packets_processed"]++;
            } else {
                statistics_["parse_errors"]++;
            }

            current += TS_PACKET_SIZE;
            processed += TS_PACKET_SIZE;
        }

        statistics_["bytes_processed"] += processed;
        return processed;
    }

    std::unordered_map<uint16_t, StreamInfo> GetStreamInfo() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stream_info_;
    }

    PATTable GetPATTable() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return pat_table_;
    }

    PMTTable GetPMTTable(uint16_t program_number) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = pmt_tables_.find(program_number);
        return (it != pmt_tables_.end()) ? it->second : PMTTable{};
    }

    void SetPIDFilter(uint16_t pid, bool enable)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (enable) {
            pid_filters_.insert(pid);
        } else {
            pid_filters_.erase(pid);
        }
    }

    void ClearPIDFilters()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pid_filters_.clear();
    }

    std::unordered_map<std::string, uint64_t> GetStatistics() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return statistics_;
    }

    void ResetStatistics()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        statistics_.clear();
    }

    void Reset()
    {
        pat_table_ = PATTable{};
        pmt_tables_.clear();
        stream_info_.clear();
        pes_buffers_.clear();
        continuity_counters_.clear();
        statistics_.clear();
    }

private:
    void ProcessTSPacket(const TSPacket &packet)
    {
        uint16_t pid = packet.header.pid;

        // Apply PID filtering if enabled
        if (!pid_filters_.empty() && pid_filters_.find(pid) == pid_filters_.end()) {
            return;
        }

        // Check continuity counter
        if (continuity_counters_.find(pid) != continuity_counters_.end()) {
            uint8_t expected = (continuity_counters_[pid] + 1) & 0x0F;
            if (packet.header.continuity_counter != expected && packet.header.adaptation_field_control != 0) {
                statistics_["continuity_errors"]++;
                LMTS_LOGW("Continuity error on PID %d: expected %d, got %d", pid, expected,
                          packet.header.continuity_counter);
            }
        }
        continuity_counters_[pid] = packet.header.continuity_counter;

        // Process based on PID
        if (pid == PAT_PID) {
            ProcessPAT(packet);
        } else if (IsPMTPID(pid)) {
            ProcessPMT(packet);
        } else if (IsElementaryStreamPID(pid)) {
            ProcessElementaryStream(packet);
        }
    }

    void ProcessPAT(const TSPacket &packet)
    {
        if (!packet.payload || packet.payload->Size() == 0) {
            return;
        }

        const uint8_t *data = packet.payload->Data();
        size_t size = packet.payload->Size();

        // Skip pointer field if payload unit start
        if (packet.header.payload_unit_start_indicator) {
            if (size < 1) {
                return;
            }
            uint8_t pointer = data[0];
            data += 1 + pointer;
            size -= 1 + pointer;
        }

        PATTable new_pat;
        if (PATTable::ParseFromData(data, size, new_pat)) {
            pat_table_ = new_pat;
            if (pat_callback_) {
                pat_callback_(pat_table_);
            }
            LMTS_LOGI("PAT parsed: %zu programs", pat_table_.programs.size());
        }
    }

    void ProcessPMT(const TSPacket &packet)
    {
        if (!packet.payload || packet.payload->Size() == 0) {
            return;
        }

        const uint8_t *data = packet.payload->Data();
        size_t size = packet.payload->Size();

        // Skip pointer field if payload unit start
        if (packet.header.payload_unit_start_indicator) {
            if (size < 1) {
                return;
            }
            uint8_t pointer = data[0];
            data += 1 + pointer;
            size -= 1 + pointer;
        }

        PMTTable new_pmt;
        if (PMTTable::ParseFromData(data, size, new_pmt)) {
            pmt_tables_[new_pmt.program_number] = new_pmt;

            // Update stream info
            for (const auto &stream : new_pmt.streams) {
                StreamInfo info;
                info.pid = stream.elementary_pid;
                info.stream_type = static_cast<StreamType>(stream.stream_type);
                info.codec_name = GetCodecName(info.stream_type);
                stream_info_[info.pid] = info;

                if (stream_info_callback_) {
                    stream_info_callback_(info);
                }
            }

            if (pmt_callback_) {
                pmt_callback_(new_pmt);
            }

            LMTS_LOGI("PMT parsed for program %d: %zu streams", new_pmt.program_number, new_pmt.streams.size());
        }
    }

    void ProcessElementaryStream(const TSPacket &packet)
    {
        uint16_t pid = packet.header.pid;

        if (!packet.payload || packet.payload->Size() == 0) {
            return;
        }

        // Accumulate PES data
        if (packet.header.payload_unit_start_indicator) {
            // Start of new PES packet
            if (pes_buffers_.find(pid) != pes_buffers_.end() && pes_buffers_[pid]->Size() > 0) {
                // Process previous PES packet
                ProcessPESPacket(pid, pes_buffers_[pid]);
            }
            pes_buffers_[pid] = std::make_shared<lmcore::DataBuffer>();
        }

        if (pes_buffers_.find(pid) != pes_buffers_.end()) {
            pes_buffers_[pid]->Append(packet.payload->Data(), packet.payload->Size());
        }
    }

    void ProcessPESPacket(uint16_t pid, std::shared_ptr<lmcore::DataBuffer> pes_data)
    {
        if (!pes_data || pes_data->Size() < 6) {
            return;
        }

        const uint8_t *data = pes_data->Data();
        size_t size = pes_data->Size();

        // Check PES start code
        if (data[0] != 0x00 || data[1] != 0x00 || data[2] != 0x01) {
            return;
        }

        // Parse PES header
        uint8_t stream_id = data[3];
        uint16_t pes_length = lmcore::ByteOrder::ReadBE16(&data[4]);

        size_t header_offset = 6;
        uint64_t timestamp = 0;

        // Parse optional PES header
        if (size > header_offset + 2) {
            uint8_t flags = data[header_offset + 1];
            uint8_t header_length = data[header_offset + 2];

            if (flags & 0x80) { // PTS present
                if (size >= header_offset + 3 + 5) {
                    const uint8_t *pts_data = &data[header_offset + 3];
                    timestamp =
                        ((static_cast<uint64_t>(pts_data[0] & 0x0E) << 29) |
                         (static_cast<uint64_t>(pts_data[1]) << 22) |
                         (static_cast<uint64_t>(pts_data[2] & 0xFE) << 14) | (static_cast<uint64_t>(pts_data[3]) << 7) |
                         (static_cast<uint64_t>(pts_data[4] & 0xFE) >> 1));
                }
            }

            header_offset += 3 + header_length;
        }

        // Extract payload
        if (header_offset < size) {
            const uint8_t *payload = data + header_offset;
            size_t payload_size = size - header_offset;

            if (stream_data_callback_) {
                stream_data_callback_(pid, payload, payload_size, timestamp);
            }

            statistics_["pes_packets_processed"]++;
        }
    }

    bool IsPMTPID(uint16_t pid) const
    {
        for (const auto &program : pat_table_.programs) {
            if (program.program_map_pid == pid) {
                return true;
            }
        }
        return false;
    }

    bool IsElementaryStreamPID(uint16_t pid) const { return stream_info_.find(pid) != stream_info_.end(); }

    std::string GetCodecName(StreamType stream_type) const
    {
        switch (stream_type) {
            case StreamType::MPEG1_VIDEO:
                return "MPEG-1 Video";
            case StreamType::MPEG2_VIDEO:
                return "MPEG-2 Video";
            case StreamType::MPEG1_AUDIO:
                return "MPEG-1 Audio";
            case StreamType::MPEG2_AUDIO:
                return "MPEG-2 Audio";
            case StreamType::AAC_AUDIO:
                return "AAC Audio";
            case StreamType::H264_VIDEO:
                return "H.264 Video";
            case StreamType::H265_VIDEO:
                return "H.265 Video";
            case StreamType::AC3_AUDIO:
                return "AC-3 Audio";
            case StreamType::EAC3_AUDIO:
                return "E-AC-3 Audio";
            default:
                return "Unknown";
        }
    }

private:
    mutable std::mutex mutex_;
    std::atomic<bool> running_;

    // Callbacks
    StreamDataCallback stream_data_callback_;
    StreamInfoCallback stream_info_callback_;
    PATCallback pat_callback_;
    PMTCallback pmt_callback_;

    // Tables and stream info
    PATTable pat_table_;
    std::unordered_map<uint16_t, PMTTable> pmt_tables_;
    std::unordered_map<uint16_t, StreamInfo> stream_info_;

    // PES buffers
    std::unordered_map<uint16_t, std::shared_ptr<lmcore::DataBuffer>> pes_buffers_;

    // State tracking
    std::unordered_map<uint16_t, uint8_t> continuity_counters_;
    std::set<uint16_t> pid_filters_;
    std::unordered_map<std::string, uint64_t> statistics_;
};

// TSDemuxer implementation
TSDemuxer::TSDemuxer() : impl_(std::make_unique<TSDemuxerImpl>()) {}

TSDemuxer::~TSDemuxer() = default;

TSDemuxer::TSDemuxer(TSDemuxer &&) noexcept = default;

TSDemuxer &TSDemuxer::operator=(TSDemuxer &&) noexcept = default;

void TSDemuxer::SetStreamDataCallback(StreamDataCallback callback)
{
    impl_->SetStreamDataCallback(std::move(callback));
}

void TSDemuxer::SetStreamInfoCallback(StreamInfoCallback callback)
{
    impl_->SetStreamInfoCallback(std::move(callback));
}

void TSDemuxer::SetPATCallback(PATCallback callback)
{
    impl_->SetPATCallback(std::move(callback));
}

void TSDemuxer::SetPMTCallback(PMTCallback callback)
{
    impl_->SetPMTCallback(std::move(callback));
}

bool TSDemuxer::Start()
{
    return impl_->Start();
}

void TSDemuxer::Stop()
{
    impl_->Stop();
}

bool TSDemuxer::IsRunning() const
{
    return impl_->IsRunning();
}

size_t TSDemuxer::ParseData(const uint8_t *data, size_t size)
{
    return impl_->ParseData(data, size);
}

std::unordered_map<uint16_t, StreamInfo> TSDemuxer::GetStreamInfo() const
{
    return impl_->GetStreamInfo();
}

PATTable TSDemuxer::GetPATTable() const
{
    return impl_->GetPATTable();
}

PMTTable TSDemuxer::GetPMTTable(uint16_t program_number) const
{
    return impl_->GetPMTTable(program_number);
}

void TSDemuxer::SetPIDFilter(uint16_t pid, bool enable)
{
    impl_->SetPIDFilter(pid, enable);
}

void TSDemuxer::ClearPIDFilters()
{
    impl_->ClearPIDFilters();
}

std::unordered_map<std::string, uint64_t> TSDemuxer::GetStatistics() const
{
    return impl_->GetStatistics();
}

void TSDemuxer::ResetStatistics()
{
    impl_->ResetStatistics();
}

void TSDemuxer::Reset()
{
    impl_->Reset();
}

} // namespace lmshao::lmts