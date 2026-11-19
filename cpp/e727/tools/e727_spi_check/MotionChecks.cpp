/**
 * @file MotionChecks.cpp
 * @brief Tier 3 checks that characterise motion: axis mapping, scale and readback latency
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include "Checks.hpp"
#include "E727SpiLink.hpp"
#include "Report.hpp"
#include "Session.hpp"
#include "Statistics.hpp"

namespace e727::spi_check {

namespace {

// Full triangle cycles used to identify a physical axis, slow enough to see on the camera
constexpr double kAxisCycleSeconds = 2.0;
constexpr double kAxisUpdateHz = 200.0;

// Fraction of the span kept clear at each end of the scale sweep, so no point sits on a limit
constexpr double kScaleSweepMarginFraction = 0.04;
constexpr int kScaleSweepPoints = 11;
constexpr int kScaleSamplesPerPoint = 8;
constexpr double kUnitySlopeTolerance = 0.05;

// Fraction of a commanded step that the readback must cover to count as having responded
constexpr double kLatencyDetectionFraction = 0.1;

/// Unit-amplitude triangle over one cycle, starting at zero so the mirror eases away from centre
[[nodiscard]] double triangleWave(double phase) noexcept {
    constexpr double kQuarter = 0.25;
    constexpr double kThreeQuarters = 0.75;
    constexpr double kSlope = 4.0;

    if (phase < kQuarter) {
        return kSlope * phase;
    }
    if (phase < kThreeQuarters) {
        return 2.0 - (kSlope * phase);
    }
    return (kSlope * phase) - kSlope;
}

[[nodiscard]] std::vector<Milliradians> axisTrajectory(const Options& options) {
    const auto updateCount = static_cast<std::size_t>(options.durationSeconds * kAxisUpdateHz);
    const double center = options.range.center().x;

    std::vector<Milliradians> trajectory;
    trajectory.reserve(updateCount);
    for (std::size_t step = 0; step < updateCount; ++step) {
        const double elapsed = static_cast<double>(step) / kAxisUpdateHz;
        const double phase = std::fmod(elapsed / kAxisCycleSeconds, 1.0);
        const double value = center + (options.amplitudeOrDefault() * triangleWave(phase));
        trajectory.push_back(positionOn(options.axis, value, options.range));
    }
    return trajectory;
}

[[nodiscard]] std::vector<Milliradians> scaleTrajectory(const Options& options) {
    const double margin = options.range.span() * kScaleSweepMarginFraction;
    const double low = options.range.min + margin;
    const double high = options.range.max - margin;
    const double step = (high - low) / static_cast<double>(kScaleSweepPoints - 1);

    std::vector<Milliradians> trajectory;
    trajectory.reserve(kScaleSweepPoints);
    for (int point = 0; point < kScaleSweepPoints; ++point) {
        trajectory.push_back(
            positionOn(options.axis, low + (step * static_cast<double>(point)), options.range));
    }
    return trajectory;
}

/// Command `target` repeatedly and return the mean readback on the driven axis, if any arrived
[[nodiscard]] std::optional<double> sampleReadback(SpiController& controller, Milliradians target,
                                                   WireAxis axis) {
    double total = 0.0;
    int validSamples = 0;
    for (int sample = 0; sample < kScaleSamplesPerPoint; ++sample) {
        if (const auto reply = controller.moveTo(target)) {
            total += valueOn(axis, reply->position);
            ++validSamples;
        }
    }
    if (validSamples == 0) {
        return std::nullopt;
    }
    return total / static_cast<double>(validSamples);
}

void writeScaleCsv(const std::filesystem::path& path, std::span<const ScalePoint> points) {
    std::ofstream file{path};
    if (!file) {
        eprintln("Failed to open {} for writing", path.string());
        return;
    }
    file << "commanded_mrad,readback_mrad\n";
    for (const ScalePoint& point : points) {
        file << std::format("{},{}\n", point.commanded, point.readback);
    }
    println("Wrote {} rows to {}", points.size(), path.string());
}

}  // namespace

int runAxisMap(const Options& options) {
    CheckReport report{"Tier 3: physical axis identification"};

    // Build the whole trajectory first, so an unsafe amplitude is caught before the mirror moves
    const std::vector<Milliradians> trajectory = axisTrajectory(options);
    if (!ensureTrajectoryWithinRange(options.range, trajectory)) {
        return EXIT_FAILURE;
    }

    println("Driving wire axis {} with a +/- {} mrad triangle for {} s.", axisLabel(options.axis),
            options.amplitudeOrDefault(), options.durationSeconds);
    println("Watch the tracking camera and record which physical axis moves.");
    if (!confirmMotion(options, "sweep one axis about the centre")) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};
    session.handshake(session.range().center());

    const auto period = std::chrono::duration<double>(1.0 / kAxisUpdateHz);
    const auto started = std::chrono::steady_clock::now();
    int transferFailures = 0;
    std::size_t stepsDone = 0;
    double windowLow = std::numeric_limits<double>::max();
    double windowHigh = std::numeric_limits<double>::lowest();
    ProgressReporter progress{"sweep", options.durationSeconds, "s"};

    for (const Milliradians& target : trajectory) {
        if (interrupted()) {
            break;
        }
        waitUntil(started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                static_cast<double>(stepsDone) * period));

        if (!session.controller().moveTo(target)) {
            ++transferFailures;
        }
        ++stepsDone;

        /*
         * Report the span covered since the last line rather than the instantaneous command. The
         * progress interval can alias with the triangle period and land on the centre crossing
         * every time, which would make a moving mirror look stationary
         */
        const double commanded = valueOn(options.axis, target);
        windowLow = std::min(windowLow, commanded);
        windowHigh = std::max(windowHigh, commanded);

        const double elapsed = static_cast<double>(stepsDone) / kAxisUpdateHz;
        if (progress.update(elapsed, std::format("spanning {:.4f} to {:.4f} mrad, failures {}",
                                                 windowLow, windowHigh, transferFailures))) {
            windowLow = commanded;
            windowHigh = commanded;
        }
    }
    progress.finish(static_cast<double>(stepsDone) / kAxisUpdateHz,
                    std::format("failures {}", transferFailures));
    session.park();

    report.check(transferFailures == 0,
                 std::format("every transfer completed ({} failed)", transferFailures));
    if (interrupted()) {
        report.warn("interrupted before the sweep finished");
    }
    report.note("Record which physical axis moved for this wire axis");

    return report.finish();
}

int runScale(const Options& options) {
    CheckReport report{"Tier 3: position scale calibration"};

    const std::vector<Milliradians> trajectory = scaleTrajectory(options);
    if (!ensureTrajectoryWithinRange(options.range, trajectory)) {
        return EXIT_FAILURE;
    }
    if (!confirmMotion(options,
                       std::format("step across the range in {} points", kScaleSweepPoints))) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};
    session.handshake(trajectory.front());

    std::vector<ScalePoint> points;
    points.reserve(trajectory.size());

    for (std::size_t index = 0; index < trajectory.size(); ++index) {
        if (interrupted()) {
            println("  interrupted, stopping the sweep early");
            break;
        }

        const Milliradians target = trajectory[index];
        const double commanded = valueOn(options.axis, target);
        println("  point {} of {}: commanding {} mrad, settling...", index + 1, trajectory.size(),
                commanded);

        (void)session.controller().moveTo(target);
        std::this_thread::sleep_for(kSettleDelay);

        const std::optional<double> readback =
            sampleReadback(session.controller(), target, options.axis);
        if (!readback) {
            report.check(false, "collected readback at a sweep point");
            continue;
        }

        points.push_back({.commanded = commanded, .readback = *readback});
        println("    readback {} mrad (residual {})", *readback, *readback - commanded);
    }
    session.park();

    report.check(points.size() == trajectory.size(), "every sweep point produced a sample");

    const LinearFit fit = fitLine(points);
    println("  slope     = {}", fit.slope);
    println("  intercept = {}", fit.intercept);
    println("  max resid = {}", fit.maxResidual);

    report.check(std::abs(fit.slope - 1.0) < kUnitySlopeTolerance,
                 "readback tracks command with unity slope");
    report.note(std::format("Command and readback share the controller's unit, so a slope far from "
                            "1.0 means the controller is not configured for the [{}, {}] mrad span "
                            "this run assumes.",
                            options.range.min, options.range.max));

    if (!options.csvPath.empty()) {
        writeScaleCsv(options.csvPath, points);
    }

    return report.finish();
}

int runLatency(const Options& options) {
    CheckReport report{"Tier 3: readback latency"};

    const double startValue = options.range.center().x;
    const double steppedValue = startValue + options.amplitudeOrDefault();
    const Milliradians start = positionOn(options.axis, startValue, options.range);
    const Milliradians stepped = positionOn(options.axis, steppedValue, options.range);
    const std::array trajectory{start, stepped};

    if (!ensureTrajectoryWithinRange(options.range, trajectory)) {
        return EXIT_FAILURE;
    }
    if (!confirmMotion(options, std::format("apply a single step of {} mrad",
                                            options.amplitudeOrDefault()))) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};
    session.handshake(start);

    const int frameCount = options.frameCount.value_or(kDefaultFrameCount);
    const double detectionThreshold = std::abs(steppedValue - startValue) * kLatencyDetectionFraction;
    std::optional<int> firstRespondingFrame;

    println("Stepping from {} to {} mrad and logging {} replies (detection threshold {} mrad)",
            startValue, steppedValue, frameCount, detectionThreshold);

    for (int frameIndex = 0; frameIndex < frameCount && !interrupted(); ++frameIndex) {
        const auto reply = session.controller().moveTo(stepped);
        if (!reply) {
            continue;
        }

        const double observed = valueOn(options.axis, reply->position);
        const double movement = std::abs(observed - startValue);
        println("  frame {}: readback {} (moved {})", frameIndex, observed, movement);

        if (!firstRespondingFrame && movement > detectionThreshold) {
            firstRespondingFrame = frameIndex;
        }
    }
    session.park();

    report.check(firstRespondingFrame.has_value(),
                 std::format("the readback responded to the step within {} frames", frameCount));
    if (firstRespondingFrame) {
        println("  step first visible in reply frame {}", *firstRespondingFrame);
        report.check(*firstRespondingFrame >= 1, "the reply lags its own command by at least one frame");
        report.note("Mechanical settling is folded into this number, so treat it as an upper bound "
                    "on protocol latency rather than a clean measurement of it");
    }

    return report.finish();
}

}  // namespace e727::spi_check
