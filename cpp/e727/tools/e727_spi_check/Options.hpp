/**
 * @file Options.hpp
 * @brief Command line options for e727_spi_check
 */

#pragma once

#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "E727Protocol.hpp"

namespace e727::spi_check {

/// Wire field driven by single-axis modes: x is bytes 4-7, y is bytes 8-11
enum class WireAxis : std::uint8_t {
    X,
    Y,
};

inline constexpr double kDefaultRateHz = 500.0;
inline constexpr double kDefaultDurationSeconds = 10.0;
inline constexpr int kDefaultFrameCount = 32;

/// Default step or sweep amplitude as a fraction of the travel span, so it follows --range
inline constexpr double kDefaultAmplitudeFraction = 0.025;

/// FTDI's vendor id, shared by every FTDI part including custom-programmed boards
inline constexpr std::uint32_t kFtdiVendorId = 0x0403;

/// Command line configuration shared by every mode
struct Options {
    std::string mode;
    int deviceIndex = 0;
    WireAxis axis = WireAxis::X;
    TravelRange range = kDefaultRange;
    std::optional<double> amplitude{};
    double rateHz = kDefaultRateHz;
    double durationSeconds = kDefaultDurationSeconds;
    std::optional<int> frameCount{};
    std::filesystem::path csvPath{};
    bool assumeYes = false;
    bool allowErrorState = false;
    std::uint32_t usbVendorId = kFtdiVendorId;
    /*
     * Registered with D2XX before enumerating. A board with a custom EEPROM reports a non-standard
     * product id, and D2XX matches against a built-in table, so it enumerates as nothing at all
     * until told about the id. lsusb shows the real value
     */
    std::optional<std::uint32_t> usbProductId{};

    /// Return the step or sweep amplitude in milliradians, defaulting to a fraction of the span
    [[nodiscard]] double amplitudeOrDefault() const {
        return amplitude.value_or(range.span() * kDefaultAmplitudeFraction);
    }
};

/// Parse the arguments after the program name, or describe the first problem found
[[nodiscard]] std::expected<Options, std::string> parseOptions(
    std::span<const std::string_view> arguments);

}  // namespace e727::spi_check
