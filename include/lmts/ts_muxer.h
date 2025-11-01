/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LMSHAO_LMTS_TS_MUXER_H
#define LMSHAO_LMTS_TS_MUXER_H

#include <lmcore/data_buffer.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

#include "ts_types.h"

namespace lmshao::lmts {

// Forward declarations
class TSMuxerImpl;

// Callback function type for output data
using TSOutputCallback = std::function<void(const uint8_t *data, size_t size)>;

/**
 * @brief MPEG Transport Stream Muxer
 *
 * This class provides functionality to multiplex audio and video streams
 * into MPEG Transport Stream format. It supports various stream types
 * including H.264/H.265 video and AAC/AC3 audio.
 */
class TSMuxer {
public:
    /**
     * @brief Constructor
     */
    TSMuxer();

    /**
     * @brief Destructor
     */
    ~TSMuxer();

    // Non-copyable
    TSMuxer(const TSMuxer &) = delete;
    TSMuxer &operator=(const TSMuxer &) = delete;

    // Movable
    TSMuxer(TSMuxer &&) noexcept;
    TSMuxer &operator=(TSMuxer &&) noexcept;

    /**
     * @brief Set output callback function
     * @param callback Function to call when TS packets are generated
     */
    void SetOutputCallback(TSOutputCallback callback);

    /**
     * @brief Set transport stream ID
     * @param ts_id Transport stream identifier
     */
    void SetTransportStreamId(uint16_t ts_id);

    /**
     * @brief Set program number
     * @param program_num Program number for PMT
     */
    void SetProgramNumber(uint16_t program_num);

    /**
     * @brief Set PMT PID
     * @param pmt_pid Program Map Table PID
     */
    void SetPMTPID(uint16_t pmt_pid);

    /**
     * @brief Set PCR PID
     * @param pcr_pid Program Clock Reference PID
     */
    void SetPCRPID(uint16_t pcr_pid);

    /**
     * @brief Add video stream
     * @param pid Elementary stream PID
     * @param stream_type Video stream type (H.264, H.265, etc.)
     * @return true if successful, false otherwise
     */
    bool AddVideoStream(uint16_t pid, StreamType stream_type);

    /**
     * @brief Add audio stream
     * @param pid Elementary stream PID
     * @param stream_type Audio stream type (AAC, AC3, etc.)
     * @return true if successful, false otherwise
     */
    bool AddAudioStream(uint16_t pid, StreamType stream_type);

    /**
     * @brief Start muxing process
     * @return true if successful, false otherwise
     */
    bool Start();

    /**
     * @brief Stop muxing process
     */
    void Stop();

    /**
     * @brief Check if muxer is running
     * @return true if running, false otherwise
     */
    bool IsRunning() const;

    /**
     * @brief Mux video data
     * @param data Video data buffer
     * @param size Data size in bytes
     * @param timestamp Presentation timestamp (90kHz units)
     * @param is_key_frame Whether this is a key frame
     * @return true if successful, false otherwise
     */
    bool MuxVideoData(const uint8_t *data, size_t size, uint64_t timestamp, bool is_key_frame = false);

    /**
     * @brief Mux audio data
     * @param data Audio data buffer
     * @param size Data size in bytes
     * @param timestamp Presentation timestamp (90kHz units)
     * @return true if successful, false otherwise
     */
    bool MuxAudioData(const uint8_t *data, size_t size, uint64_t timestamp);

    /**
     * @brief Flush any pending data
     */
    void Flush();

    /**
     * @brief Get statistics
     * @return Statistics as key-value pairs
     */
    std::unordered_map<std::string, uint64_t> GetStatistics() const;

    /**
     * @brief Reset statistics
     */
    void ResetStatistics();

private:
    std::unique_ptr<TSMuxerImpl> impl_;
};

} // namespace lmshao::lmts

#endif // LMSHAO_LMTS_TS_MUXER_H