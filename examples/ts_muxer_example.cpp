/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#include <lmcore/logger.h>
#include <lmts/ts_muxer.h>
#include <lmts/ts_types.h>

#include <fstream>
#include <iostream>
#include <vector>

using namespace lmshao::lmts;

int main(int argc, char *argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_h264_file> <output_ts_file>" << std::endl;
        return 1;
    }

    const char *input_file = argv[1];
    const char *output_file = argv[2];

    // Open input file
    std::ifstream input(input_file, std::ios::binary);
    if (!input) {
        std::cerr << "Failed to open input file: " << input_file << std::endl;
        return 1;
    }

    // Open output file
    std::ofstream output(output_file, std::ios::binary);
    if (!output) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return 1;
    }

    // Create TS muxer
    TSMuxer muxer;

    // Set output callback
    muxer.SetOutputCallback(
        [&output](const uint8_t *data, size_t size) { output.write(reinterpret_cast<const char *>(data), size); });

    // Configure muxer
    muxer.SetTransportStreamId(1);
    muxer.SetProgramNumber(1);
    muxer.SetPMTPID(0x1000);

    // Add video stream
    if (!muxer.AddVideoStream(0x100, StreamType::H264_VIDEO)) {
        std::cerr << "Failed to add video stream" << std::endl;
        return 1;
    }

    muxer.SetPCRPID(0x100);

    // Start muxing
    if (!muxer.Start()) {
        std::cerr << "Failed to start muxer" << std::endl;
        return 1;
    }

    std::cout << "Muxing started..." << std::endl;

    // Read and mux data
    const size_t buffer_size = 4096;
    std::vector<uint8_t> buffer(buffer_size);
    uint64_t timestamp = 0;
    size_t total_bytes = 0;

    while (input.read(reinterpret_cast<char *>(buffer.data()), buffer_size) || input.gcount() > 0) {
        size_t bytes_read = input.gcount();

        // Simple frame detection (look for NAL unit start codes)
        bool is_key_frame = false;
        if (bytes_read >= 4) {
            // Check for SPS (0x67) which indicates a key frame
            for (size_t i = 0; i < bytes_read - 4; ++i) {
                if (buffer[i] == 0x00 && buffer[i + 1] == 0x00 && buffer[i + 2] == 0x00 && buffer[i + 3] == 0x01 &&
                    (buffer[i + 4] & 0x1F) == 0x07) {
                    is_key_frame = true;
                    break;
                }
            }
        }

        if (!muxer.MuxVideoData(buffer.data(), bytes_read, timestamp, is_key_frame)) {
            std::cerr << "Failed to mux video data" << std::endl;
            break;
        }

        timestamp += 3000; // Assume 30fps (90kHz / 30 = 3000)
        total_bytes += bytes_read;

        if (total_bytes % (1024 * 1024) == 0) {
            std::cout << "Processed " << (total_bytes / 1024 / 1024) << " MB" << std::endl;
        }
    }

    // Flush any remaining data
    muxer.Flush();

    // Stop muxer
    muxer.Stop();

    // Print statistics
    auto stats = muxer.GetStatistics();
    std::cout << "\nMuxing completed!" << std::endl;
    std::cout << "Statistics:" << std::endl;
    for (const auto &[key, value] : stats) {
        std::cout << "  " << key << ": " << value << std::endl;
    }

    return 0;
}