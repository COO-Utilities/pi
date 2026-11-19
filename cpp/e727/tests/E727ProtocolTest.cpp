/**
 * @file E727ProtocolTest.cpp
 * @brief Checks of E727Protocol.hpp beyond the frozen CRC vectors in the header itself
 *
 * Everything expressible as a constant expression is a static_assert, so a regression fails the
 * build. The few checks that need runtime (string formatting, a numeric sweep) run from main.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <format>
#include <iostream>
#include <limits>
#include <string_view>

#include "E727Protocol.hpp"

namespace e727::test {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

constexpr Milliradians kProbe{.x = 20.125, .y = 14.875};
constexpr Frame kProbeFrame = buildFrame(Servo::Closed, kProbe);

[[nodiscard]] constexpr Frame withByteFlipped(Frame frame, std::size_t index) {
    frame[index] ^= 0x01U;
    return frame;
}

// Layout
static_assert(kProbeFrame[kProtocolIdByte] == kProtocolId);
static_assert(kProbeFrame[kAxisCountByte] == kAxisCount);
static_assert(kProbeFrame[kFlagsByte] == kNoFlags);
static_assert(servoOf(kProbeFrame) == Servo::Closed);
static_assert(servoOf(buildFrame(Servo::Open, kProbe)) == Servo::Open);

// 17.5f is 0x418C0000, so a big-endian store puts 0x41 first
static_assert(buildFrame(Servo::Open, {.x = 17.5, .y = 0.0})[kPositionXByte] == 0x41);
static_assert(buildFrame(Servo::Open, {.x = 17.5, .y = 0.0})[kPositionXByte + 1] == 0x8C);

// Values exactly representable in binary32 survive the round trip unchanged
static_assert(positionOf(kProbeFrame) == kProbe);
static_assert(buildFrame(Servo::Closed, {.x = kProbe.y, .y = kProbe.x}) != kProbeFrame);

// Checksum validation
static_assert(isValid(kProbeFrame));
static_assert(!isValid(withByteFlipped(kProbeFrame, kFrameSize - 1)));
static_assert(!isValid(withByteFlipped(kProbeFrame, kPositionXByte)));
static_assert(!isValid(withByteFlipped(kProbeFrame, kServoByte)));
static_assert(decode(kProbeFrame).checksumValid);
static_assert(decode(kProbeFrame).position == kProbe);

// Units
static_assert(toMicroradians(Milliradians{.x = 17.5, .y = 1.0}) == Microradians{.x = 17'500.0, .y = 1'000.0});
static_assert(toMilliradians(Microradians{.x = 500.0, .y = 0.0}) == Milliradians{.x = 0.5, .y = 0.0});

// Travel range
static_assert(kDefaultRange.center() == Milliradians{.x = 17.5, .y = 17.5});
static_assert(kDefaultRange.span() == 35.0);
static_assert(kDefaultRange.contains(Milliradians{.x = 0.0, .y = 35.0}));
static_assert(!kDefaultRange.contains(Milliradians{.x = -0.001, .y = 17.5}));
static_assert(!kDefaultRange.contains(Milliradians{.x = 17.5, .y = 35.001}));
static_assert(!kDefaultRange.contains(Milliradians{.x = kNaN, .y = 17.5}));
static_assert(!TravelRange{.min = 5.0, .max = 5.0}.isValid());
static_assert(!TravelRange{.min = 10.0, .max = 5.0}.isValid());
static_assert(!TravelRange{.min = 0.0, .max = kInfinity}.isValid());
static_assert(!TravelRange{.min = kNaN, .max = 5.0}.isValid());
static_assert(TravelRange{.min = -2.0, .max = 2.0}.center() == Milliradians{.x = 0.0, .y = 0.0});

/// Count runtime failures so every check reports before the test exits
class Checker {
public:
    void expect(bool condition, std::string_view description) {
        std::cout << std::format("  [{}] {}\n", condition ? "PASS" : "FAIL", description);
        failures_ += condition ? 0 : 1;
    }

    [[nodiscard]] int exitCode() const noexcept { return failures_ == 0 ? EXIT_SUCCESS : EXIT_FAILURE; }

private:
    int failures_ = 0;
};

void checkHexFormatting(Checker& checker) {
    const std::string_view expected = "10 02 00 03 41 8C 00 00 41 8C 00 00 0A 2B";
    checker.expect(toHex(buildFrame(Servo::Closed, kDefaultRange.center())) == expected,
                   "toHex formats the centre frame as space-separated uppercase bytes");
}

// Sweep the whole default range rather than trusting a single probe value
void checkConversionRoundTrip(Checker& checker) {
    constexpr int kSteps = 10'000;
    constexpr double kUlpAllowance = 8.0;
    const double tolerance = kUlpAllowance * kDefaultRange.max * std::numeric_limits<double>::epsilon();

    double worstError = 0.0;
    for (int step = 0; step <= kSteps; ++step) {
        const double value = kDefaultRange.min + (kDefaultRange.span() * step / kSteps);
        const Milliradians original{.x = value, .y = value};
        const Milliradians roundTrip = toMilliradians(toMicroradians(original));
        worstError = std::max(worstError, std::abs(roundTrip.x - original.x));
    }
    checker.expect(worstError <= tolerance,
                   std::format("mrad to urad round trip stays within {:.3e} (worst {:.3e})",
                               tolerance, worstError));
}

}  // namespace

}  // namespace e727::test

int main() {
    e727::test::Checker checker;
    e727::test::checkHexFormatting(checker);
    e727::test::checkConversionRoundTrip(checker);
    return checker.exitCode();
}
