/**
 * @file TimingChecks.cpp
 * @brief Tier 3 checks that characterise the link: paced rate, unpaced speed and error soak
 *
 * Each holds the mirror still at the centre under servo control while frames are exchanged, so
 * these are safe to run for a long time even though they drive the servo.
 */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "Checks.hpp"
#include "E727SpiLink.hpp"
#include "Report.hpp"
#include "Session.hpp"
#include "Statistics.hpp"

namespace e727::spi_check {

namespace {

// Rate the tracking loop runs at, used to express speed results as headroom
constexpr double kTrackingLoopHz = 500.0;

// Frames in an unpaced burst, large enough for percentile tails to mean something
constexpr int kDefaultSpeedFrames = 15'000;

// Frames between progress polls in a burst, to keep clock reads out of the measurement
constexpr int kProgressStride = 64;

constexpr double kMicrosecondsPerSecond = 1e6;

using Microseconds = std::chrono::duration<double, std::micro>;

/// Counts and timings gathered while exchanging frames at a fixed cadence
struct SustainedStats {
    std::vector<double> transferMicroseconds;
    int transferFailures = 0;
    int checksumFailures = 0;
    long long framesAttempted = 0;
};

/// Everything gathered during an unpaced burst
struct BurstStats {
    std::vector<double> transferMicroseconds;
    // Interval between consecutive transfer starts, so one entry shorter than the above
    std::vector<double> periodMicroseconds;
    double elapsedSeconds = 0.0;
    int transferFailures = 0;
    int checksumFailures = 0;
    int longestFailureRun = 0;
};

[[nodiscard]] std::string failureDetail(int transferFailures, int checksumFailures) {
    return std::format("failed {}, bad checksum {}", transferFailures, checksumFailures);
}

/// Re-command the centre at a fixed cadence for a fixed duration, timing every transfer
[[nodiscard]] SustainedStats runSustainedTransfers(MotionSession& session, double rateHz,
                                                   double durationSeconds) {
    SustainedStats stats;
    const auto totalFrames = static_cast<long long>(rateHz * durationSeconds);
    stats.transferMicroseconds.reserve(static_cast<std::size_t>(totalFrames));

    const Frame command = buildFrame(Servo::Closed, session.range().center());
    const auto period = std::chrono::duration<double>(1.0 / rateHz);
    const auto started = std::chrono::steady_clock::now();

    println("Holding position while transferring {} frames at {} Hz ({} s). Ctrl-C stops early and "
            "parks the mirror.",
            totalFrames, rateHz, durationSeconds);
    ProgressReporter progress{"transfers", static_cast<double>(totalFrames), "frames"};

    for (long long frame = 0; frame < totalFrames && !interrupted(); ++frame) {
        waitUntil(started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                static_cast<double>(frame) * period));

        const auto transferStart = std::chrono::steady_clock::now();
        const auto reply = session.controller().exchange(command);
        const auto transferEnd = std::chrono::steady_clock::now();

        ++stats.framesAttempted;
        stats.transferMicroseconds.push_back(Microseconds{transferEnd - transferStart}.count());
        if (!reply) {
            ++stats.transferFailures;
        } else if (!isValid(*reply)) {
            ++stats.checksumFailures;
        }

        progress.update(static_cast<double>(stats.framesAttempted),
                        failureDetail(stats.transferFailures, stats.checksumFailures));
    }

    progress.finish(static_cast<double>(stats.framesAttempted),
                    failureDetail(stats.transferFailures, stats.checksumFailures));
    if (interrupted()) {
        println("  interrupted after {} frames", stats.framesAttempted);
    }
    return stats;
}

/**
 * Send frames back to back with no pacing, to find the link's ceiling.
 *
 * Distinct from runSustainedTransfers, which holds a fixed rate to prove a cadence is achievable.
 * Here there is no wait between frames, so the achieved rate is whatever the link sustains.
 */
[[nodiscard]] BurstStats runBurst(MotionSession& session, int frameCount) {
    BurstStats stats;
    stats.transferMicroseconds.reserve(static_cast<std::size_t>(frameCount));
    stats.periodMicroseconds.reserve(static_cast<std::size_t>(frameCount));

    // Build the frame once so that construction cost never lands inside a measured interval
    const Frame command = buildFrame(Servo::Closed, session.range().center());

    println("Sending {} frames back to back with no pacing. Ctrl-C stops early and parks the mirror.",
            frameCount);
    ProgressReporter progress{"burst", static_cast<double>(frameCount), "frames"};

    int currentFailureRun = 0;
    const auto burstStart = std::chrono::steady_clock::now();
    auto previousStart = burstStart;

    for (int frame = 0; frame < frameCount && !interrupted(); ++frame) {
        const auto transferStart = std::chrono::steady_clock::now();
        const auto reply = session.controller().exchange(command);
        const auto transferEnd = std::chrono::steady_clock::now();

        stats.transferMicroseconds.push_back(Microseconds{transferEnd - transferStart}.count());
        if (frame > 0) {
            stats.periodMicroseconds.push_back(Microseconds{transferStart - previousStart}.count());
        }
        previousStart = transferStart;

        if (!reply) {
            ++stats.transferFailures;
            stats.longestFailureRun = std::max(stats.longestFailureRun, ++currentFailureRun);
        } else {
            currentFailureRun = 0;
            if (!isValid(*reply)) {
                ++stats.checksumFailures;
            }
        }

        /*
         * Poll the progress reporter only every kProgressStride frames. It reads the clock, and
         * that read sits inside the start-to-start interval this mode reports as period jitter,
         * so polling every frame would measure our own instrumentation
         */
        if (frame % kProgressStride == 0) {
            progress.update(static_cast<double>(frame + 1),
                            failureDetail(stats.transferFailures, stats.checksumFailures));
        }
    }

    stats.elapsedSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - burstStart).count();
    progress.finish(static_cast<double>(stats.transferMicroseconds.size()),
                    failureDetail(stats.transferFailures, stats.checksumFailures));
    return stats;
}

void writeTimingCsv(const std::filesystem::path& path, const SustainedStats& stats) {
    std::ofstream file{path};
    if (!file) {
        eprintln("Failed to open {} for writing", path.string());
        return;
    }
    file << "frame,transfer_microseconds\n";
    for (std::size_t i = 0; i < stats.transferMicroseconds.size(); ++i) {
        file << std::format("{},{}\n", i, stats.transferMicroseconds[i]);
    }
    println("Wrote {} rows to {}", stats.transferMicroseconds.size(), path.string());
}

void writeBurstCsv(const std::filesystem::path& path, const BurstStats& stats) {
    std::ofstream file{path};
    if (!file) {
        eprintln("Failed to open {} for writing", path.string());
        return;
    }
    file << "frame,transfer_microseconds,period_microseconds\n";
    for (std::size_t i = 0; i < stats.transferMicroseconds.size(); ++i) {
        // The first frame has no earlier transfer to measure a period against
        const std::string period = i > 0 ? std::format("{}", stats.periodMicroseconds[i - 1]) : "";
        file << std::format("{},{},{}\n", i, stats.transferMicroseconds[i], period);
    }
    println("Wrote {} rows to {}", stats.transferMicroseconds.size(), path.string());
}

void printBurstRates(const BurstStats& stats, const TimingSummary& period, double achievedHz) {
    /*
     * Two different rates, both worth seeing. Throughput is wall clock and so includes this tool's
     * own progress printing, while the ceiling from the mean period is what the link would sustain
     * if nothing else ran between frames
     */
    const double ceilingHz = period.mean > 0.0 ? kMicrosecondsPerSecond / period.mean
                                               : std::numeric_limits<double>::infinity();

    println();
    println("  frames sent    = {}", stats.transferMicroseconds.size());
    println("  elapsed        = {} s", stats.elapsedSeconds);
    println("  throughput     = {} Hz (wall clock)", achievedHz);
    println("  ceiling        = {} Hz (from mean period)", ceilingHz);
    println("  mean period    = {} us", period.mean);
    println("  period jitter  = {} us stddev", period.standardDeviation);

    // Peak-to-peak spread bounds the worst cadence slip, which matters more to a fixed-rate loop
    if (period.count > 0) {
        println("  period spread  = {} us peak to peak", period.max - period.min);
    }
    println();
}

}  // namespace

int runRate(const Options& options) {
    CheckReport report{"Tier 3: sustained transfer timing"};

    if (!confirmMotion(options,
                       std::format("hold the centre while re-commanding at {} Hz", options.rateHz))) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};
    session.handshake(session.range().center());
    const SustainedStats stats =
        runSustainedTransfers(session, options.rateHz, options.durationSeconds);
    session.park();

    const TimingSummary summary = summarize(stats.transferMicroseconds);
    printTimingSummary("transfer duration", summary);

    report.check(stats.transferFailures == 0,
                 std::format("every transfer completed ({} failed)", stats.transferFailures));

    const double budgetMicroseconds = kMicrosecondsPerSecond / options.rateHz;
    report.check(summary.count > 0 && summary.p99 < budgetMicroseconds,
                 std::format("p99 transfer time fits inside the {} us cadence budget",
                             budgetMicroseconds));

    if (!options.csvPath.empty()) {
        writeTimingCsv(options.csvPath, stats);
    }
    return report.finish();
}

int runSpeed(const Options& options) {
    CheckReport report{"Tier 3: unpaced speed and reliability"};

    const int frameCount = options.frameCount.value_or(kDefaultSpeedFrames);
    if (!confirmMotion(options, std::format("hold the centre while sending {} frames as fast as the "
                                            "link allows",
                                            frameCount))) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};
    session.handshake(session.range().center());
    const BurstStats stats = runBurst(session, frameCount);
    session.park();

    if (stats.transferMicroseconds.empty()) {
        report.check(false, "the burst produced at least one transfer");
        return report.finish();
    }

    const TimingSummary transfer = summarize(stats.transferMicroseconds);
    const TimingSummary period = summarize(stats.periodMicroseconds);
    const double achievedHz =
        static_cast<double>(stats.transferMicroseconds.size()) / stats.elapsedSeconds;

    printBurstRates(stats, period, achievedHz);
    printTimingSummary("transfer duration", transfer);
    println();
    printTimingSummary("loop period", period);
    println();
    reportOutliers(stats.transferMicroseconds, transfer);
    println();
    reportHistogram(stats.transferMicroseconds, transfer);
    println();

    println("  reliability");
    println("    transfer failures   = {}", stats.transferFailures);
    println("    checksum failures   = {}", stats.checksumFailures);
    println("    longest failure run = {} consecutive frames", stats.longestFailureRun);
    println();

    report.check(stats.transferMicroseconds.size() == static_cast<std::size_t>(frameCount),
                 std::format("all {} frames were sent", frameCount));
    report.check(stats.transferFailures == 0,
                 std::format("no transfer failures ({} seen)", stats.transferFailures));
    report.check(stats.checksumFailures == 0,
                 std::format("no reply checksum failures ({} seen)", stats.checksumFailures));
    report.check(achievedHz > kTrackingLoopHz,
                 std::format("achieved rate clears the {} Hz tracking loop", kTrackingLoopHz));
    report.note(std::format("headroom over the tracking loop: {:.2f}x", achievedHz / kTrackingLoopHz));

    if (!options.csvPath.empty()) {
        writeBurstCsv(options.csvPath, stats);
    }
    return report.finish();
}

int runSoak(const Options& options) {
    CheckReport report{"Tier 3: error soak"};

    if (!confirmMotion(options, std::format("hold the centre for {} s while counting link errors",
                                            options.durationSeconds))) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};
    session.handshake(session.range().center());
    const SustainedStats stats =
        runSustainedTransfers(session, options.rateHz, options.durationSeconds);
    session.park();

    println("  frames attempted  = {}", stats.framesAttempted);
    println("  transfer failures = {}", stats.transferFailures);
    println("  checksum failures = {}", stats.checksumFailures);

    report.check(stats.transferFailures == 0, "no transfer failures over the soak");
    report.check(stats.checksumFailures == 0, "no reply checksum failures over the soak");
    if (stats.checksumFailures > 0) {
        report.note("Non-zero checksum failures make reply validation in the tracking loop a "
                    "requirement rather than a defensive extra");
    }

    return report.finish();
}

}  // namespace e727::spi_check
