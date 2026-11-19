/**
 * @file Session.hpp
 * @brief Hardware sequences, safety checks and interrupt handling shared by e727_spi_check modes
 */

#pragma once

#include <chrono>
#include <span>
#include <string_view>

#include "E727SpiLink.hpp"
#include "Options.hpp"

namespace e727::spi_check {

/// Settle time after a servo state change or a large move
inline constexpr auto kSettleDelay = std::chrono::milliseconds(330);

/// Make Ctrl-C request a clean stop instead of killing the process mid-motion
void installInterruptHandler();

[[nodiscard]] bool interrupted() noexcept;

/// Hold until `deadline` in short sleeps, so the cadence stays tight and Ctrl-C stays responsive
void waitUntil(std::chrono::steady_clock::time_point deadline);

/// Ask before moving the mirror, unless --yes was given
[[nodiscard]] bool confirmMotion(const Options& options, std::string_view description);

/// Refuse a position outside `range` rather than clamping it, explaining why on stderr
[[nodiscard]] bool ensureWithinRange(const TravelRange& range, Milliradians position,
                                     std::string_view context);

/// Validate a whole trajectory before the first frame goes out, never mid-sweep
[[nodiscard]] bool ensureTrajectoryWithinRange(const TravelRange& range,
                                               std::span<const Milliradians> trajectory);

[[nodiscard]] std::string_view axisLabel(WireAxis axis) noexcept;

[[nodiscard]] double valueOn(WireAxis axis, Milliradians position) noexcept;

/// Return a position with `axis` at `value` and the other axis held at the centre of `range`
[[nodiscard]] Milliradians positionOn(WireAxis axis, double value, const TravelRange& range) noexcept;

/// Open the FT4222 for `options` and log the settings that can skew results
[[nodiscard]] SpiController openController(const Options& options);

/// Controller for a mode that moves the mirror, parked at the centre with loops open when it ends
class MotionSession {
public:
    explicit MotionSession(const Options& options);
    ~MotionSession();

    MotionSession(const MotionSession&) = delete;
    MotionSession& operator=(const MotionSession&) = delete;
    MotionSession(MotionSession&&) = delete;
    MotionSession& operator=(MotionSession&&) = delete;

    [[nodiscard]] SpiController& controller() noexcept { return controller_; }
    [[nodiscard]] const TravelRange& range() const noexcept { return controller_.range(); }

    /// Settle at the centre in open loop, for modes that must not engage the servo
    void settleOpenLoop();

    /// Settle at the centre in open loop, then close the servo loops at `start`
    void handshake(Milliradians start);

    /// Return to the centre under servo control, then open the loops; later calls do nothing
    void park();

private:
    void settleAt(Milliradians target);

    SpiController controller_;
    bool parked_ = false;
};

}  // namespace e727::spi_check
