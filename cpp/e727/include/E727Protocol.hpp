/**
 * @file E727Protocol.hpp
 * @brief Units, travel range and SPI frame format for the PI E-727 fast steering mirror controller
 *
 * Pure computation with no hardware dependency: everything here works in constant expressions and
 * builds without the FTDI SDK. The FT4222 transport is in E727SpiLink.hpp.
 *
 * The wire carries positions in whatever unit and span the controller is configured for. The
 * HISPEC E-727 is configured for 0 to 35 milliradians, so Milliradians is the native type and
 * kDefaultRange matches that span. A controller configured differently needs a different
 * TravelRange, and a frame built for one configuration commands a different angle on another.
 */

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <span>
#include <string>
#include <utility>

namespace e727 {

// Units ---------------------------------------------------------------------------------------

/**
 * Position of both axes in milliradians, the unit the wire carries.
 *
 * x and y follow wire order. Which physical axis each one drives is established with
 * `e727_spi_check axis`; confirm it before relying on the names.
 */
struct Milliradians {
    double x = 0.0;
    double y = 0.0;

    friend constexpr bool operator==(const Milliradians&, const Milliradians&) = default;
};

/// Position of both axes in microradians, converted to Milliradians before it reaches the wire
struct Microradians {
    double x = 0.0;
    double y = 0.0;

    friend constexpr bool operator==(const Microradians&, const Microradians&) = default;
};

inline constexpr double kMicroradiansPerMilliradian = 1'000.0;

[[nodiscard]] constexpr Milliradians toMilliradians(Microradians position) noexcept {
    return {.x = position.x / kMicroradiansPerMilliradian,
            .y = position.y / kMicroradiansPerMilliradian};
}

[[nodiscard]] constexpr Microradians toMicroradians(Milliradians position) noexcept {
    return {.x = position.x * kMicroradiansPerMilliradian,
            .y = position.y * kMicroradiansPerMilliradian};
}

// Travel range --------------------------------------------------------------------------------

/**
 * Inclusive travel limits in milliradians, applied to both axes.
 *
 * Commands outside the range are refused, never clamped, so a bad request cannot be silently
 * reshaped into a different motion. NaN is outside every range.
 */
struct TravelRange {
    double min = 0.0;
    double max = 0.0;

    /// Return whether both limits are finite and `min` lies strictly below `max`
    [[nodiscard]] constexpr bool isValid() const noexcept {
        constexpr double kLowest = std::numeric_limits<double>::lowest();
        constexpr double kHighest = std::numeric_limits<double>::max();
        return kLowest <= min && min < max && max <= kHighest;
    }

    [[nodiscard]] constexpr bool contains(double milliradians) const noexcept {
        return milliradians >= min && milliradians <= max;
    }

    [[nodiscard]] constexpr bool contains(Milliradians position) const noexcept {
        return contains(position.x) && contains(position.y);
    }

    [[nodiscard]] constexpr double span() const noexcept { return max - min; }

    [[nodiscard]] constexpr Milliradians center() const noexcept {
        const double middle = min + (span() / 2.0);
        return {.x = middle, .y = middle};
    }

    friend constexpr bool operator==(const TravelRange&, const TravelRange&) = default;
};

/// Closed-loop span of the HISPEC E-727 as currently configured
inline constexpr TravelRange kDefaultRange{.min = 0.0, .max = 35.0};

// Frame format --------------------------------------------------------------------------------
//
// One 14-byte frame each way per transaction, same layout in both directions:
//
//   byte  0      protocol id
//   byte  1      axis count
//   byte  2      flags, none used
//   byte  3      servo state
//   bytes 4-7    x position, IEEE-754 binary32, big-endian
//   bytes 8-11   y position, same encoding
//   bytes 12-13  CRC-16 over bytes 0-11, big-endian

inline constexpr std::size_t kFrameSize = 14;
using Frame = std::array<std::uint8_t, kFrameSize>;

inline constexpr std::size_t kProtocolIdByte = 0;
inline constexpr std::size_t kAxisCountByte = 1;
inline constexpr std::size_t kFlagsByte = 2;
inline constexpr std::size_t kServoByte = 3;
inline constexpr std::size_t kPositionXByte = 4;
inline constexpr std::size_t kPositionYByte = 8;
inline constexpr std::size_t kChecksumByte = 12;
inline constexpr std::size_t kChecksumCoverage = kChecksumByte;

inline constexpr std::uint8_t kProtocolId = 0x10;
inline constexpr std::uint8_t kAxisCount = 0x02;
inline constexpr std::uint8_t kNoFlags = 0x00;

enum class Servo : std::uint8_t {
    Open = 0x00,
    Closed = 0x03,
};

/// Decoded content of a reply frame
struct Reply {
    Milliradians position;
    /**
     * Whether the reply's checksum matched, so `position` can be trusted.
     *
     * Reported rather than enforced: the reply checksum convention is not yet confirmed on
     * hardware, and enforcing a wrong guess would turn every healthy transfer into a failure.
     */
    bool checksumValid = false;
};

// Checksum ------------------------------------------------------------------------------------

inline constexpr std::uint16_t kCrcInit = 0xFFFF;
inline constexpr std::uint16_t kCrcPolynomial = 0x8005;

/// Compute the E-727 CRC-16: polynomial 0x8005, init 0xFFFF, unreflected, no final XOR
[[nodiscard]] constexpr std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept {
    constexpr std::uint16_t kTopBit = 0x8000;
    constexpr int kBitsPerByte = std::numeric_limits<std::uint8_t>::digits;

    std::uint16_t crc = kCrcInit;
    for (const std::uint8_t byte : bytes) {
        crc ^= static_cast<std::uint16_t>(byte << kBitsPerByte);
        for (int bit = 0; bit < kBitsPerByte; ++bit) {
            const bool carry = (crc & kTopBit) != 0;
            crc = static_cast<std::uint16_t>(crc << 1);
            if (carry) {
                crc ^= kCrcPolynomial;
            }
        }
    }
    return crc;
}

namespace detail {

static_assert(std::numeric_limits<float>::is_iec559, "The wire carries IEEE-754 binary32");

// Store big-endian whatever the host order, so frames are identical on every machine
template <std::unsigned_integral Word>
constexpr void writeBigEndian(Frame& frame, std::size_t offset, Word word) noexcept {
    if constexpr (std::endian::native == std::endian::little) {
        word = std::byteswap(word);
    }
    const auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(Word)>>(word);
    std::ranges::copy(bytes, std::span{frame}.subspan(offset, sizeof(Word)).begin());
}

template <std::unsigned_integral Word>
[[nodiscard]] constexpr Word readBigEndian(const Frame& frame, std::size_t offset) noexcept {
    std::array<std::uint8_t, sizeof(Word)> bytes{};
    std::ranges::copy(std::span{frame}.subspan(offset, sizeof(Word)), bytes.begin());
    const auto word = std::bit_cast<Word>(bytes);
    if constexpr (std::endian::native == std::endian::little) {
        return std::byteswap(word);
    }
    return word;
}

constexpr void writeAxis(Frame& frame, std::size_t offset, double value) noexcept {
    writeBigEndian(frame, offset, std::bit_cast<std::uint32_t>(static_cast<float>(value)));
}

[[nodiscard]] constexpr double readAxis(const Frame& frame, std::size_t offset) noexcept {
    return std::bit_cast<float>(readBigEndian<std::uint32_t>(frame, offset));
}

[[nodiscard]] constexpr std::uint16_t computeChecksum(const Frame& frame) noexcept {
    return crc16(std::span{frame}.first(kChecksumCoverage));
}

// Recompute the checksum after any covered byte changes
constexpr void seal(Frame& frame) noexcept {
    writeBigEndian(frame, kChecksumByte, computeChecksum(frame));
}

}  // namespace detail

// Frames --------------------------------------------------------------------------------------

/// Build a complete command frame, checksum included
[[nodiscard]] constexpr Frame buildFrame(Servo servo, Milliradians target) noexcept {
    Frame frame{};
    frame[kProtocolIdByte] = kProtocolId;
    frame[kAxisCountByte] = kAxisCount;
    frame[kFlagsByte] = kNoFlags;
    frame[kServoByte] = std::to_underlying(servo);
    detail::writeAxis(frame, kPositionXByte, target.x);
    detail::writeAxis(frame, kPositionYByte, target.y);
    detail::seal(frame);
    return frame;
}

/// Return `frame` with a new servo state and a recomputed checksum
[[nodiscard]] constexpr Frame withServo(Frame frame, Servo servo) noexcept {
    frame[kServoByte] = std::to_underlying(servo);
    detail::seal(frame);
    return frame;
}

[[nodiscard]] constexpr Servo servoOf(const Frame& frame) noexcept {
    return static_cast<Servo>(frame[kServoByte]);
}

/// Extract both axes from a command or reply frame
[[nodiscard]] constexpr Milliradians positionOf(const Frame& frame) noexcept {
    return {.x = detail::readAxis(frame, kPositionXByte),
            .y = detail::readAxis(frame, kPositionYByte)};
}

/// Read the checksum a frame carries, without recomputing it
[[nodiscard]] constexpr std::uint16_t checksumOf(const Frame& frame) noexcept {
    return detail::readBigEndian<std::uint16_t>(frame, kChecksumByte);
}

[[nodiscard]] constexpr bool isValid(const Frame& frame) noexcept {
    return checksumOf(frame) == detail::computeChecksum(frame);
}

[[nodiscard]] constexpr Reply decode(const Frame& reply) noexcept {
    return {.position = positionOf(reply), .checksumValid = isValid(reply)};
}

/// Format a frame as space-separated hex bytes, e.g. "10 02 00 03 ..."
[[nodiscard]] inline std::string toHex(const Frame& frame) {
    std::string text;
    for (const std::uint8_t byte : frame) {
        std::format_to(std::back_inserter(text), "{}{:02X}", text.empty() ? "" : " ", byte);
    }
    return text;
}

// Frozen vectors ------------------------------------------------------------------------------
//
// Checked wherever this header is included. They come from this implementation, not a vendor
// reference, so they catch a refactor changing the CRC or the layout; only
// `e727_spi_check readback` proves the E-727 accepts them. Positions are raw wire values, several
// taken from an earlier 0 to 10000 configuration purely to pin the encoding.

namespace detail::vectors {
inline constexpr std::array<std::uint8_t, 1> kSingleByte{0x41};
inline constexpr std::array<std::uint8_t, 4> kSequence{0x01, 0x02, 0x03, 0x04};
}  // namespace detail::vectors

static_assert(crc16({}) == kCrcInit);
static_assert(crc16(detail::vectors::kSingleByte) == 0xFC84);
static_assert(crc16(detail::vectors::kSequence) == 0x9E17);
static_assert(checksumOf(buildFrame(Servo::Closed, {.x = 17.5, .y = 17.5})) == 0x0A2B);
static_assert(checksumOf(buildFrame(Servo::Closed, {.x = 5000.0, .y = 5000.0})) == 0x8A31);
static_assert(checksumOf(buildFrame(Servo::Open, {.x = 5000.0, .y = 5000.0})) == 0x8091);
static_assert(checksumOf(buildFrame(Servo::Closed, {.x = 100.0, .y = 9900.0})) == 0xBD53);
static_assert(checksumOf(buildFrame(Servo::Closed, {.x = 5250.0, .y = 4750.0})) == 0x1144);
static_assert(checksumOf(buildFrame(Servo::Closed, {.x = 0.0, .y = 0.0})) == 0x0198);
static_assert(positionOf(buildFrame(Servo::Open, kDefaultRange.center())) == kDefaultRange.center());
static_assert(servoOf(withServo(buildFrame(Servo::Open, {}), Servo::Closed)) == Servo::Closed);
static_assert(isValid(withServo(buildFrame(Servo::Open, {}), Servo::Closed)));
static_assert(kDefaultRange.isValid());

}  // namespace e727
