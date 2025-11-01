/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LMSHAO_LMTS_TS_DEMUXER_H
#define LMSHAO_LMTS_TS_DEMUXER_H

#include <lmcore/data_buffer.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

#include "ts_types.h"

namespace lmshao::lmts {

// Forward declarations
class TSDemuxerImpl;

// Stream information structure
struct StreamInfo {
    uint16_t pid;
    StreamType stream_type;
    std::string codec_name;
    std::unordered_map<std::string, std::string> metadata;

    StreamInfo() : pid(0), stream_type(StreamType::MPEG1_VIDEO) {}
};

// Callback function types
using StreamDataCallback = std::function<void(uint16_t pid, const uint8_t *data, size_t size, uint64_t timestamp)>;
using StreamInfoCallback = std::function<void(const StreamInfo &info)>;
using PATCallback = std::function<void(const PATTable &pat)>;
using PMTCallback = std::function<void(const PMTTable &pmt)>;

/**
 * @brief MPEG Transport Stream Demuxer
 *
 * This class provides functionality to demultiplex MPEG Transport Stream
 * data and extract individual audio/video streams. It supports parsing
 * of PAT/PMT tables and extraction of elementary streams.
 */
class TSDemuxer {
public:
    /**
     * @brief Constructor
     */
    TSDemuxer();

    /**
     * @brief Destructor
     */
    ~TSDemuxer();

    // Non-copyable
    TSDemuxer(const TSDemuxer &) = delete;
    TSDemuxer &operator=(const TSDemuxer &) = delete;

    // Movable
    TSDemuxer(TSDemuxer &&) noexcept;
    TSDemuxer &operator=(TSDemuxer &&) noexcept;

    /**
     * @brief Set stream data callback
     * @param callback Function to call when stream data is extracted
     */
    void SetStreamDataCallback(StreamDataCallback callback);

    /**
     * @brief Set stream info callback
     * @param callback Function to call when new stream is discovered
     */
    void SetStreamInfoCallback(StreamInfoCallback callback);

    /**
     * @brief Set PAT callback
     * @param callback Function to call when PAT is parsed
     */
    void SetPATCallback(PATCallback callback);

    /**
     * @brief Set PMT callback
     * @param callback Function to call when PMT is parsed
     */
    void SetPMTCallback(PMTCallback callback);

    /**
     * @brief Start demuxing process
     * @return true if successful, false otherwise
     */
    bool Start();

    /**
     * @brief Stop demuxing process
     */
    void Stop();

    /**
     * @brief Check if demuxer is running
     * @return true if running, false otherwise
     */
    bool IsRunning() const;

    /**
     * @brief Parse TS data
     * @param data Input TS data buffer
     * @param size Data size in bytes
     * @return Number of bytes processed
     */
    size_t ParseData(const uint8_t *data, size_t size);

    /**
     * @brief Get stream information
     * @return Map of PID to stream information
     */
    std::unordered_map<uint16_t, StreamInfo> GetStreamInfo() const;

    /**
     * @brief Get current PAT table
     * @return Current PAT table (may be empty if not parsed yet)
     */
    PATTable GetPATTable() const;

    /**
     * @brief Get current PMT table
     * @param program_number Program number to get PMT for
     * @return Current PMT table (may be empty if not parsed yet)
     */
    PMTTable GetPMTTable(uint16_t program_number) const;

    /**
     * @brief Enable/disable specific PID filtering
     * @param pid PID to filter
     * @param enable true to enable, false to disable
     */
    void SetPIDFilter(uint16_t pid, bool enable);

    /**
     * @brief Clear all PID filters
     */
    void ClearPIDFilters();

    /**
     * @brief Get statistics
     * @return Statistics as key-value pairs
     */
    std::unordered_map<std::string, uint64_t> GetStatistics() const;

    /**
     * @brief Reset statistics
     */
    void ResetStatistics();

    /**
     * @brief Reset demuxer state
     */
    void Reset();

private:
    std::unique_ptr<TSDemuxerImpl> impl_;
};

} // namespace lmshao::lmts

#endif // LMSHAO_LMTS_TS_DEMUXER_H