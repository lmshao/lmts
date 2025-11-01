/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LMSHAO_LMTS_INTERNAL_LOGGER_H
#define LMSHAO_LMTS_INTERNAL_LOGGER_H

#include <mutex>

#include "lmts/lmts_logger.h"

namespace lmshao::lmts {

/**
 * @brief Get LMTS logger with automatic initialization
 *
 * Used internally by LMTS modules.
 * Ensures the logger is initialized before first use.
 */
inline lmcore::Logger &GetLmtsLoggerWithAutoInit()
{
    static std::once_flag initFlag;
    std::call_once(initFlag, []() {
        lmcore::LoggerRegistry::RegisterModule<LmtsModuleTag>("LMTS");
        InitLmtsLogger();
    });
    return lmcore::LoggerRegistry::GetLogger<LmtsModuleTag>();
}

// Internal LMTS logging macros with auto-initialization and module tagging
#define LMTS_LOGD(fmt, ...)                                                                                            \
    do {                                                                                                               \
        auto &logger = lmshao::lmts::GetLmtsLoggerWithAutoInit();                                                      \
        if (logger.ShouldLog(lmcore::LogLevel::kDebug)) {                                                              \
            logger.LogWithModuleTag<lmshao::lmts::LmtsModuleTag>(lmcore::LogLevel::kDebug, __FILE__, __LINE__,         \
                                                                 __FUNCTION__, fmt, ##__VA_ARGS__);                    \
        }                                                                                                              \
    } while (0)

#define LMTS_LOGI(fmt, ...)                                                                                            \
    do {                                                                                                               \
        auto &logger = lmshao::lmts::GetLmtsLoggerWithAutoInit();                                                      \
        if (logger.ShouldLog(lmcore::LogLevel::kInfo)) {                                                               \
            logger.LogWithModuleTag<lmshao::lmts::LmtsModuleTag>(lmcore::LogLevel::kInfo, __FILE__, __LINE__,          \
                                                                 __FUNCTION__, fmt, ##__VA_ARGS__);                    \
        }                                                                                                              \
    } while (0)

#define LMTS_LOGW(fmt, ...)                                                                                            \
    do {                                                                                                               \
        auto &logger = lmshao::lmts::GetLmtsLoggerWithAutoInit();                                                      \
        if (logger.ShouldLog(lmcore::LogLevel::kWarn)) {                                                               \
            logger.LogWithModuleTag<lmshao::lmts::LmtsModuleTag>(lmcore::LogLevel::kWarn, __FILE__, __LINE__,          \
                                                                 __FUNCTION__, fmt, ##__VA_ARGS__);                    \
        }                                                                                                              \
    } while (0)

#define LMTS_LOGE(fmt, ...)                                                                                            \
    do {                                                                                                               \
        auto &logger = lmshao::lmts::GetLmtsLoggerWithAutoInit();                                                      \
        if (logger.ShouldLog(lmcore::LogLevel::kError)) {                                                              \
            logger.LogWithModuleTag<lmshao::lmts::LmtsModuleTag>(lmcore::LogLevel::kError, __FILE__, __LINE__,         \
                                                                 __FUNCTION__, fmt, ##__VA_ARGS__);                    \
        }                                                                                                              \
    } while (0)

#define LMTS_LOGF(fmt, ...)                                                                                            \
    do {                                                                                                               \
        auto &logger = lmshao::lmts::GetLmtsLoggerWithAutoInit();                                                      \
        if (logger.ShouldLog(lmcore::LogLevel::kFatal)) {                                                              \
            logger.LogWithModuleTag<lmshao::lmts::LmtsModuleTag>(lmcore::LogLevel::kFatal, __FILE__, __LINE__,         \
                                                                 __FUNCTION__, fmt, ##__VA_ARGS__);                    \
        }                                                                                                              \
    } while (0)

} // namespace lmshao::lmts

#endif // LMSHAO_LMTS_INTERNAL_LOGGER_H