/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#include <lmcore/logger.h>
#include <lmts/ts_demuxer.h>
#include <lmts/ts_types.h>

#include <fstream>
#include <iostream>
#include <map>
#include <vector>

using namespace lmshao::lmts;

class MediaFileExtractor {
private:
    std::ofstream video_file_;
    std::ofstream audio_file_;
    std::map<uint16_t, uint8_t> stream_types_; // PID -> StreamType
    bool video_file_opened_;
    bool audio_file_opened_;
    size_t video_packets_;
    size_t audio_packets_;
    size_t total_bytes_written_;
    std::string video_filename_;
    std::string audio_filename_;

public:
    MediaFileExtractor()
        : video_file_opened_(false), audio_file_opened_(false), video_packets_(0), audio_packets_(0),
          total_bytes_written_(0)
    {
    }

    ~MediaFileExtractor()
    {
        if (video_file_.is_open()) {
            video_file_.close();
            std::cout << "Video file closed: " << video_filename_ << std::endl;
        }
        if (audio_file_.is_open()) {
            audio_file_.close();
            std::cout << "Audio file closed: " << audio_filename_ << std::endl;
        }
    }

    std::string GetVideoExtension(uint8_t stream_type) const
    {
        switch (stream_type) {
            case static_cast<uint8_t>(StreamType::H264_VIDEO):
                return ".h264";
            case static_cast<uint8_t>(StreamType::H265_VIDEO):
                return ".h265";
            case static_cast<uint8_t>(StreamType::MPEG1_VIDEO):
            case static_cast<uint8_t>(StreamType::MPEG2_VIDEO):
                return ".m2v";
            default:
                return ".video";
        }
    }

    std::string GetAudioExtension(uint8_t stream_type) const
    {
        switch (stream_type) {
            case static_cast<uint8_t>(StreamType::AAC_AUDIO):
                return ".aac";
            case static_cast<uint8_t>(StreamType::AC3_AUDIO):
                return ".ac3";
            case static_cast<uint8_t>(StreamType::EAC3_AUDIO):
                return ".eac3";
            case static_cast<uint8_t>(StreamType::MPEG1_AUDIO):
            case static_cast<uint8_t>(StreamType::MPEG2_AUDIO):
                return ".mp3";
            default:
                return ".audio";
        }
    }

    void SetStreamTypes(const std::map<uint16_t, uint8_t> &stream_types) { stream_types_ = stream_types; }

    void OnVideoData(uint16_t pid, const uint8_t *data, size_t size, uint64_t pts)
    {
        if (!video_file_opened_) {
            // Determine filename based on stream type
            auto it = stream_types_.find(pid);
            if (it != stream_types_.end()) {
                video_filename_ = "output_video" + GetVideoExtension(it->second);
            } else {
                video_filename_ = "output_video.raw"; // Default extension
            }

            video_file_.open(video_filename_, std::ios::binary);
            if (video_file_.is_open()) {
                video_file_opened_ = true;
                std::cout << "Video file opened: " << video_filename_ << std::endl;
            } else {
                std::cerr << "Failed to open video file: " << video_filename_ << std::endl;
                return;
            }
        }

        if (video_file_.is_open() && data && size > 0) {
            video_file_.write(reinterpret_cast<const char *>(data), size);
            video_packets_++;
            total_bytes_written_ += size;

            if (video_packets_ % 100 == 0) {
                std::cout << "Video packets: " << video_packets_ << ", PTS: " << pts << ", Size: " << size << " bytes"
                          << std::endl;
            }
        }
    }

    void OnAudioData(uint16_t pid, const uint8_t *data, size_t size, uint64_t pts)
    {
        if (!audio_file_opened_) {
            // Determine filename based on stream type
            auto it = stream_types_.find(pid);
            if (it != stream_types_.end()) {
                audio_filename_ = "output_audio" + GetAudioExtension(it->second);
            } else {
                audio_filename_ = "output_audio.raw"; // Default extension
            }

            audio_file_.open(audio_filename_, std::ios::binary);
            if (audio_file_.is_open()) {
                audio_file_opened_ = true;
                std::cout << "Audio file opened: " << audio_filename_ << std::endl;
            } else {
                std::cerr << "Failed to open audio file: " << audio_filename_ << std::endl;
                return;
            }
        }

        if (audio_file_.is_open() && data && size > 0) {
            audio_file_.write(reinterpret_cast<const char *>(data), size);
            audio_packets_++;
            total_bytes_written_ += size;

            if (audio_packets_ % 100 == 0) {
                std::cout << "Audio packets: " << audio_packets_ << ", PTS: " << pts << ", Size: " << size << " bytes"
                          << std::endl;
            }
        }
    }

    void PrintStatistics() const
    {
        std::cout << "\n=== Media Extraction Statistics ===" << std::endl;
        std::cout << "Video packets extracted: " << video_packets_ << std::endl;
        std::cout << "Audio packets extracted: " << audio_packets_ << std::endl;
        std::cout << "Total bytes written: " << total_bytes_written_ << std::endl;
        if (video_file_opened_) {
            std::cout << "Video file: " << video_filename_ << std::endl;
        }
        if (audio_file_opened_) {
            std::cout << "Audio file: " << audio_filename_ << std::endl;
        }
    }
};

bool IsVideoStream(uint8_t stream_type)
{
    return stream_type == static_cast<uint8_t>(StreamType::H264_VIDEO) ||
           stream_type == static_cast<uint8_t>(StreamType::H265_VIDEO) ||
           stream_type == static_cast<uint8_t>(StreamType::MPEG1_VIDEO) ||
           stream_type == static_cast<uint8_t>(StreamType::MPEG2_VIDEO);
}

bool IsAudioStream(uint8_t stream_type)
{
    return stream_type == static_cast<uint8_t>(StreamType::AAC_AUDIO) ||
           stream_type == static_cast<uint8_t>(StreamType::AC3_AUDIO) ||
           stream_type == static_cast<uint8_t>(StreamType::EAC3_AUDIO) ||
           stream_type == static_cast<uint8_t>(StreamType::MPEG1_AUDIO) ||
           stream_type == static_cast<uint8_t>(StreamType::MPEG2_AUDIO);
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input_ts_file>" << std::endl;
        return 1;
    }

    const char *input_file = argv[1];

    // Open input file
    std::ifstream input(input_file, std::ios::binary);
    if (!input) {
        std::cerr << "Failed to open input file: " << input_file << std::endl;
        return 1;
    }

    // Create TS demuxer and media extractor
    TSDemuxer demuxer;
    MediaFileExtractor extractor;
    std::map<uint16_t, uint8_t> discovered_streams;

    // Set callbacks
    demuxer.SetPATCallback([](const PATTable &pat) {
        std::cout << "PAT received:" << std::endl;
        std::cout << "  Transport Stream ID: " << pat.transport_stream_id << std::endl;
        std::cout << "  Version: " << static_cast<int>(pat.version_number) << std::endl;
        std::cout << "  Programs: " << pat.programs.size() << std::endl;
        for (const auto &program : pat.programs) {
            std::cout << "    Program " << program.program_number << " -> PMT PID: 0x" << std::hex
                      << program.program_map_pid << std::dec << std::endl;
        }
    });

    demuxer.SetPMTCallback([&](const PMTTable &pmt) {
        std::cout << "PMT received for program " << pmt.program_number << ":" << std::endl;
        std::cout << "  PCR PID: 0x" << std::hex << pmt.pcr_pid << std::dec << std::endl;
        std::cout << "  Streams: " << pmt.streams.size() << std::endl;

        // Store stream types for file extension determination
        for (const auto &stream : pmt.streams) {
            discovered_streams[stream.elementary_pid] = stream.stream_type;
            std::cout << "    PID: 0x" << std::hex << stream.elementary_pid << std::dec << ", Type: 0x" << std::hex
                      << static_cast<int>(stream.stream_type) << std::dec;

            if (IsVideoStream(stream.stream_type)) {
                std::cout << " (Video)";
            } else if (IsAudioStream(stream.stream_type)) {
                std::cout << " (Audio)";
            }
            std::cout << std::endl;
        }

        // Update extractor with stream types
        extractor.SetStreamTypes(discovered_streams);
    });

    demuxer.SetStreamInfoCallback([](const StreamInfo &info) {
        std::cout << "Stream discovered:" << std::endl;
        std::cout << "  PID: 0x" << std::hex << info.pid << std::dec << std::endl;
        std::cout << "  Codec: " << info.codec_name << std::endl;
    });

    demuxer.SetStreamDataCallback([&](uint16_t pid, const uint8_t *data, size_t size, uint64_t timestamp) {
        // Check if this is a known stream
        auto it = discovered_streams.find(pid);
        if (it != discovered_streams.end()) {
            if (IsVideoStream(it->second)) {
                extractor.OnVideoData(pid, data, size, timestamp);
            } else if (IsAudioStream(it->second)) {
                extractor.OnAudioData(pid, data, size, timestamp);
            }
        }
    });

    // Start demuxing
    if (!demuxer.Start()) {
        std::cerr << "Failed to start demuxer" << std::endl;
        return 1;
    }

    std::cout << "Demuxing started..." << std::endl;

    // Read and parse data
    const size_t buffer_size = 188 * 7; // Multiple of TS packet size
    std::vector<uint8_t> buffer(buffer_size);
    size_t total_bytes = 0;

    while (input.read(reinterpret_cast<char *>(buffer.data()), buffer_size) || input.gcount() > 0) {
        size_t bytes_read = input.gcount();
        size_t processed = demuxer.ParseData(buffer.data(), bytes_read);

        total_bytes += processed;

        if (total_bytes % (1024 * 1024) == 0) {
            std::cout << "Processed " << (total_bytes / 1024 / 1024) << " MB" << std::endl;
        }
    }

    // Stop demuxer
    demuxer.Stop();

    // Print final statistics
    auto stats = demuxer.GetStatistics();
    std::cout << "\nDemuxing completed!" << std::endl;
    std::cout << "Demuxer Statistics:" << std::endl;
    for (const auto &[key, value] : stats) {
        std::cout << "  " << key << ": " << value << std::endl;
    }

    // Print stream information
    auto stream_info = demuxer.GetStreamInfo();
    std::cout << "\nDiscovered streams:" << std::endl;
    for (const auto &[pid, info] : stream_info) {
        std::cout << "  PID: 0x" << std::hex << pid << std::dec << " - " << info.codec_name << std::endl;
    }

    // Print extraction statistics
    extractor.PrintStatistics();

    return 0;
}