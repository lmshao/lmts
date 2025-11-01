/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#include <lmcore/byte_order.h>
#include <lmts/ts_muxer.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>

#include "internal_logger.h"

namespace lmshao::lmts {

// Stream information for muxing
struct StreamInfo {
    uint16_t pid;
    StreamType stream_type;
    uint8_t continuity_counter;
    uint64_t last_timestamp;

    StreamInfo() : pid(0), stream_type(StreamType::MPEG1_VIDEO), continuity_counter(0), last_timestamp(0) {}
};

class TSMuxerImpl {
public:
    TSMuxerImpl() : running_(false), transport_stream_id_(1), program_number_(1), pmt_pid_(PMT_PID), pcr_pid_(0) {}

    ~TSMuxerImpl() { Stop(); }

    void SetOutputCallback(TSOutputCallback callback) { output_callback_ = std::move(callback); }

    void SetTransportStreamId(uint16_t ts_id) { transport_stream_id_ = ts_id; }

    void SetProgramNumber(uint16_t program_num) { program_number_ = program_num; }

    void SetPMTPID(uint16_t pmt_pid) { pmt_pid_ = pmt_pid; }

    void SetPCRPID(uint16_t pcr_pid) { pcr_pid_ = pcr_pid; }

    bool AddVideoStream(uint16_t pid, StreamType stream_type)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (streams_.find(pid) != streams_.end()) {
            LMTS_LOGE("Stream with PID %d already exists", pid);
            return false;
        }

        StreamInfo info;
        info.pid = pid;
        info.stream_type = stream_type;
        streams_[pid] = info;

        // Set PCR PID to first video stream if not set
        if (pcr_pid_ == 0) {
            pcr_pid_ = pid;
        }

        LMTS_LOGI("Added video stream: PID=%d, type=%d", pid, static_cast<int>(stream_type));
        return true;
    }

    bool AddAudioStream(uint16_t pid, StreamType stream_type)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (streams_.find(pid) != streams_.end()) {
            LMTS_LOGE("Stream with PID %d already exists", pid);
            return false;
        }

        StreamInfo info;
        info.pid = pid;
        info.stream_type = stream_type;
        streams_[pid] = info;

        LMTS_LOGI("Added audio stream: PID=%d, type=%d", pid, static_cast<int>(stream_type));
        return true;
    }

    bool Start()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (running_) {
            LMTS_LOGW("Muxer is already running");
            return true;
        }

        if (!output_callback_) {
            LMTS_LOGE("Output callback not set");
            return false;
        }

        if (streams_.empty()) {
            LMTS_LOGE("No streams added");
            return false;
        }

        running_ = true;
        pat_counter_ = 0;
        pmt_counter_ = 0;

        // Send initial PAT and PMT
        SendPAT();
        SendPMT();

        LMTS_LOGI("TS Muxer started");
        return true;
    }

    void Stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        LMTS_LOGI("TS Muxer stopped");
    }

    bool IsRunning() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return running_;
    }

    bool MuxVideoData(const uint8_t *data, size_t size, uint64_t timestamp, bool is_key_frame)
    {
        return MuxStreamData(data, size, timestamp, true, is_key_frame);
    }

    bool MuxAudioData(const uint8_t *data, size_t size, uint64_t timestamp)
    {
        return MuxStreamData(data, size, timestamp, false, false);
    }

    void Flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Send PAT/PMT periodically
        SendPAT();
        SendPMT();
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

private:
    bool MuxStreamData(const uint8_t *data, size_t size, uint64_t timestamp, bool is_video, bool is_key_frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!running_) {
            LMTS_LOGE("Muxer is not running");
            return false;
        }

        // Find appropriate stream
        uint16_t target_pid = 0;
        for (auto &[pid, info] : streams_) {
            bool is_video_stream =
                (info.stream_type == StreamType::H264_VIDEO || info.stream_type == StreamType::H265_VIDEO ||
                 info.stream_type == StreamType::MPEG1_VIDEO || info.stream_type == StreamType::MPEG2_VIDEO);

            if (is_video == is_video_stream) {
                target_pid = pid;
                break;
            }
        }

        if (target_pid == 0) {
            LMTS_LOGE("No suitable stream found for data");
            return false;
        }

        auto &stream_info = streams_[target_pid];

        // Create PES packet
        std::vector<uint8_t> pes_packet = CreatePESPacket(data, size, timestamp, is_video);

        // Fragment into TS packets
        size_t offset = 0;
        bool first_packet = true;

        while (offset < pes_packet.size()) {
            TSPacket ts_packet;
            ts_packet.header.pid = target_pid;
            ts_packet.header.payload_unit_start_indicator = first_packet ? 1 : 0;
            ts_packet.header.adaptation_field_control = 1; // Payload only
            ts_packet.header.continuity_counter = stream_info.continuity_counter;

            // Add PCR if this is the PCR PID and key frame
            if (target_pid == pcr_pid_ && is_key_frame && first_packet) {
                AddPCRToPacket(ts_packet, timestamp);
            }

            size_t payload_size = std::min(pes_packet.size() - offset, TS_PAYLOAD_SIZE);
            if (ts_packet.adaptation_field) {
                payload_size = std::min(payload_size, TS_PAYLOAD_SIZE - ts_packet.adaptation_field->Size());
            }

            ts_packet.payload = std::make_shared<lmcore::DataBuffer>(payload_size);
            ts_packet.payload->Append(pes_packet.data() + offset, payload_size);

            // Pad with stuffing bytes if needed
            if (ts_packet.payload->Size() < TS_PAYLOAD_SIZE) {
                size_t padding = TS_PAYLOAD_SIZE - ts_packet.payload->Size();
                if (ts_packet.adaptation_field) {
                    padding -= ts_packet.adaptation_field->Size();
                }
                if (padding > 0) {
                    std::vector<uint8_t> stuffing(padding, 0xFF);
                    ts_packet.payload->Append(stuffing.data(), stuffing.size());
                }
            }

            // Send packet
            auto raw_data = ts_packet.GetRawData();
            output_callback_(raw_data.data(), raw_data.size());

            stream_info.continuity_counter = (stream_info.continuity_counter + 1) & 0x0F;
            offset += payload_size;
            first_packet = false;
        }

        // Update statistics
        statistics_["packets_sent"]++;
        statistics_["bytes_sent"] += size;

        return true;
    }

    std::vector<uint8_t> CreatePESPacket(const uint8_t *data, size_t size, uint64_t timestamp, bool is_video)
    {
        std::vector<uint8_t> pes_packet;

        // PES header
        pes_packet.push_back(0x00); // Packet start code prefix
        pes_packet.push_back(0x00);
        pes_packet.push_back(0x01);

        // Stream ID
        pes_packet.push_back(is_video ? 0xE0 : 0xC0);

        // PES packet length (0 for video, actual length for audio)
        uint16_t pes_length = is_video ? 0 : (size + 8); // 8 bytes for PES header extension
        pes_packet.push_back((pes_length >> 8) & 0xFF);
        pes_packet.push_back(pes_length & 0xFF);

        // PES header extension
        pes_packet.push_back(0x80); // '10' + scrambling + priority + alignment +
                                    // copyright + original
        pes_packet.push_back(0x80); // PTS flag set
        pes_packet.push_back(0x05); // PES header data length

        // PTS (33 bits)
        uint64_t pts = timestamp & 0x1FFFFFFFFULL;
        pes_packet.push_back(0x21 | ((pts >> 29) & 0x0E));
        pes_packet.push_back((pts >> 22) & 0xFF);
        pes_packet.push_back(0x01 | ((pts >> 14) & 0xFE));
        pes_packet.push_back((pts >> 7) & 0xFF);
        pes_packet.push_back(0x01 | ((pts << 1) & 0xFE));

        // Payload
        pes_packet.insert(pes_packet.end(), data, data + size);

        return pes_packet;
    }

    void AddPCRToPacket(TSPacket &packet, uint64_t timestamp)
    {
        // Create adaptation field with PCR
        std::vector<uint8_t> af_data;
        af_data.push_back(7);    // Adaptation field length
        af_data.push_back(0x10); // PCR flag set

        // PCR (42 bits)
        uint64_t pcr_base = timestamp / 300;
        uint16_t pcr_ext = timestamp % 300;

        af_data.push_back((pcr_base >> 25) & 0xFF);
        af_data.push_back((pcr_base >> 17) & 0xFF);
        af_data.push_back((pcr_base >> 9) & 0xFF);
        af_data.push_back((pcr_base >> 1) & 0xFF);
        af_data.push_back(((pcr_base & 0x01) << 7) | 0x7E | ((pcr_ext >> 8) & 0x01));
        af_data.push_back(pcr_ext & 0xFF);

        packet.adaptation_field = std::make_shared<lmcore::DataBuffer>(af_data.size());
        packet.adaptation_field->Append(af_data.data(), af_data.size());
        packet.header.adaptation_field_control = 3; // Both adaptation field and payload
    }

    void SendPAT()
    {
        std::vector<uint8_t> pat_data;

        // PAT table header
        pat_data.push_back(PAT_TABLE_ID);
        pat_data.push_back(0xB0); // Section syntax indicator + reserved + section length high
        pat_data.push_back(13);   // Section length low (fixed for single program)

        // Transport stream ID
        pat_data.push_back((transport_stream_id_ >> 8) & 0xFF);
        pat_data.push_back(transport_stream_id_ & 0xFF);

        // Version and flags
        pat_data.push_back(0xC1); // Reserved + version + current/next
        pat_data.push_back(0x00); // Section number
        pat_data.push_back(0x00); // Last section number

        // Program entry
        pat_data.push_back((program_number_ >> 8) & 0xFF);
        pat_data.push_back(program_number_ & 0xFF);
        pat_data.push_back(0xE0 | ((pmt_pid_ >> 8) & 0x1F));
        pat_data.push_back(pmt_pid_ & 0xFF);

        // CRC32 (simplified - should be calculated properly)
        pat_data.push_back(0x00);
        pat_data.push_back(0x00);
        pat_data.push_back(0x00);
        pat_data.push_back(0x00);

        SendTableData(PAT_PID, pat_data, pat_counter_);
    }

    void SendPMT()
    {
        std::vector<uint8_t> pmt_data;

        // PMT table header
        pmt_data.push_back(PMT_TABLE_ID);

        // Calculate section length
        uint16_t section_length = 9 + (streams_.size() * 5) + 4; // Header + streams + CRC
        pmt_data.push_back(0xB0 | ((section_length >> 8) & 0x0F));
        pmt_data.push_back(section_length & 0xFF);

        // Program number
        pmt_data.push_back((program_number_ >> 8) & 0xFF);
        pmt_data.push_back(program_number_ & 0xFF);

        // Version and flags
        pmt_data.push_back(0xC1);
        pmt_data.push_back(0x00); // Section number
        pmt_data.push_back(0x00); // Last section number

        // PCR PID
        pmt_data.push_back(0xE0 | ((pcr_pid_ >> 8) & 0x1F));
        pmt_data.push_back(pcr_pid_ & 0xFF);

        // Program info length (0)
        pmt_data.push_back(0xF0);
        pmt_data.push_back(0x00);

        // Stream entries
        for (const auto &[pid, info] : streams_) {
            pmt_data.push_back(static_cast<uint8_t>(info.stream_type));
            pmt_data.push_back(0xE0 | ((pid >> 8) & 0x1F));
            pmt_data.push_back(pid & 0xFF);
            pmt_data.push_back(0xF0); // ES info length high
            pmt_data.push_back(0x00); // ES info length low
        }

        // CRC32 (simplified)
        pmt_data.push_back(0x00);
        pmt_data.push_back(0x00);
        pmt_data.push_back(0x00);
        pmt_data.push_back(0x00);

        SendTableData(pmt_pid_, pmt_data, pmt_counter_);
    }

    void SendTableData(uint16_t pid, const std::vector<uint8_t> &table_data, uint8_t &counter)
    {
        TSPacket packet;
        packet.header.pid = pid;
        packet.header.payload_unit_start_indicator = 1;
        packet.header.adaptation_field_control = 1;
        packet.header.continuity_counter = counter;

        // Add pointer field for PSI tables
        std::vector<uint8_t> payload_data;
        payload_data.push_back(0x00); // Pointer field
        payload_data.insert(payload_data.end(), table_data.begin(), table_data.end());

        // Pad to TS payload size
        while (payload_data.size() < TS_PAYLOAD_SIZE) {
            payload_data.push_back(0xFF);
        }

        packet.payload = std::make_shared<lmcore::DataBuffer>(payload_data.size());
        packet.payload->Append(payload_data.data(), payload_data.size());

        auto raw_data = packet.GetRawData();
        output_callback_(raw_data.data(), raw_data.size());

        counter = (counter + 1) & 0x0F;
    }

private:
    mutable std::mutex mutex_;
    std::atomic<bool> running_;
    TSOutputCallback output_callback_;

    uint16_t transport_stream_id_;
    uint16_t program_number_;
    uint16_t pmt_pid_;
    uint16_t pcr_pid_;

    std::unordered_map<uint16_t, StreamInfo> streams_;
    std::unordered_map<std::string, uint64_t> statistics_;

    uint8_t pat_counter_;
    uint8_t pmt_counter_;
};

// TSMuxer implementation
TSMuxer::TSMuxer() : impl_(std::make_unique<TSMuxerImpl>()) {}

TSMuxer::~TSMuxer() = default;

TSMuxer::TSMuxer(TSMuxer &&) noexcept = default;

TSMuxer &TSMuxer::operator=(TSMuxer &&) noexcept = default;

void TSMuxer::SetOutputCallback(TSOutputCallback callback)
{
    impl_->SetOutputCallback(std::move(callback));
}

void TSMuxer::SetTransportStreamId(uint16_t ts_id)
{
    impl_->SetTransportStreamId(ts_id);
}

void TSMuxer::SetProgramNumber(uint16_t program_num)
{
    impl_->SetProgramNumber(program_num);
}

void TSMuxer::SetPMTPID(uint16_t pmt_pid)
{
    impl_->SetPMTPID(pmt_pid);
}

void TSMuxer::SetPCRPID(uint16_t pcr_pid)
{
    impl_->SetPCRPID(pcr_pid);
}

bool TSMuxer::AddVideoStream(uint16_t pid, StreamType stream_type)
{
    return impl_->AddVideoStream(pid, stream_type);
}

bool TSMuxer::AddAudioStream(uint16_t pid, StreamType stream_type)
{
    return impl_->AddAudioStream(pid, stream_type);
}

bool TSMuxer::Start()
{
    return impl_->Start();
}

void TSMuxer::Stop()
{
    impl_->Stop();
}

bool TSMuxer::IsRunning() const
{
    return impl_->IsRunning();
}

bool TSMuxer::MuxVideoData(const uint8_t *data, size_t size, uint64_t timestamp, bool is_key_frame)
{
    return impl_->MuxVideoData(data, size, timestamp, is_key_frame);
}

bool TSMuxer::MuxAudioData(const uint8_t *data, size_t size, uint64_t timestamp)
{
    return impl_->MuxAudioData(data, size, timestamp);
}

void TSMuxer::Flush()
{
    impl_->Flush();
}

std::unordered_map<std::string, uint64_t> TSMuxer::GetStatistics() const
{
    return impl_->GetStatistics();
}

void TSMuxer::ResetStatistics()
{
    impl_->ResetStatistics();
}

} // namespace lmshao::lmts