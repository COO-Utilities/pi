/**
 * @file Checks.hpp
 * @brief Entry points for each e727_spi_check mode, grouped by hardware tier
 *
 * Each returns a process exit status. Tier 0 (frame layout and CRC vectors) has no entry point:
 * it runs as static_asserts in E727Protocol.hpp, so it passed if this tool compiled.
 */

#pragma once

#include "Options.hpp"

namespace e727::spi_check {

// Tier 1: FT4222 present, controller absent
[[nodiscard]] int runProbe(const Options& options);
[[nodiscard]] int runLoopback(const Options& options);

// Tier 2: controller present, no motion beyond recentring
[[nodiscard]] int runReadback(const Options& options);
[[nodiscard]] int runBadChecksum(const Options& options);

// Tier 3: controller present, mirror moves
[[nodiscard]] int runAxisMap(const Options& options);
[[nodiscard]] int runScale(const Options& options);
[[nodiscard]] int runLatency(const Options& options);
[[nodiscard]] int runRate(const Options& options);
[[nodiscard]] int runSpeed(const Options& options);
[[nodiscard]] int runSoak(const Options& options);

}  // namespace e727::spi_check
