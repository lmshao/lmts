/**
 * @author SHAO Liming <lmshao@163.com>
 * @copyright Copyright (c) 2025 SHAO Liming
 * @license MIT
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef LMSHAO_LMTS_LMTS_LOGGER_H
#define LMSHAO_LMTS_LMTS_LOGGER_H

#include <lmcore/logger.h>

namespace lmshao::lmts {

// Module tag for Lmts
struct LmtsModuleTag {};

/**
 * @brief Initialize Lmts logger with specified settings
 * @param level Log level (default: Debug in debug builds, Warn in release
 * builds)
 * @param output Output destination (default: CONSOLE)
 * @param filename Log file name (optional)
 */
inline void InitLmtsLogger(lmcore::LogLevel level =
#if defined(_DEBUG) || defined(DEBUG) || !defined(NDEBUG)
                               lmcore::LogLevel::kDebug,
#else
                               lmcore::LogLevel::kWarn,
#endif
                           lmcore::LogOutput output = lmcore::LogOutput::CONSOLE, const std::string &filename = "")
{
    // Register module if not already registered
    lmcore::LoggerRegistry::RegisterModule<LmtsModuleTag>("LMTS");
    lmcore::LoggerRegistry::InitLogger<LmtsModuleTag>(level, output, filename);
}

} // namespace lmshao::lmts

#endif // LMSHAO_LMTS_LMTS_LOGGER_H