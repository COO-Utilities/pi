/**
 * @file QuietChecks.cpp
 * @brief Tier 2 checks: E-727 present, no motion beyond recentring
 */

#include <algorithm>
#include <cstdlib>
#include <expected>
#include <format>
#include <string>

#include "Checks.hpp"
#include "E727SpiLink.hpp"
#include "Report.hpp"
#include "Session.hpp"

namespace e727::spi_check {

namespace {

// Frames dumped as hex before switching to summary counts
constexpr int kFramesToDump = 4;

constexpr std::uint8_t kStuckLow = 0x00;
constexpr std::uint8_t kStuckHigh = 0xFF;

// Flag reply patterns that indicate the controller is not driving MISO at all
void reportSuspiciousReply(const CheckReport& report, const Frame& command, const Frame& reply) {
    const auto allBytesAre = [&reply](std::uint8_t value) {
        return std::ranges::all_of(reply, [value](std::uint8_t byte) { return byte == value; });
    };

    if (allBytesAre(kStuckLow) || allBytesAre(kStuckHigh)) {
        report.warn(std::format("reply is entirely 0x{:02X}, which usually means MISO is stuck or "
                                "the controller is not powered",
                                reply.front()));
    }
    if (reply == command) {
        report.warn("reply is byte-identical to the command, so MISO may still be jumpered to MOSI "
                    "from the loopback test");
    }
}

[[nodiscard]] std::string showReply(const std::expected<Frame, Error>& reply) {
    return reply ? toHex(*reply) : std::format("<{}>", e727::describe(reply.error()));
}

}  // namespace

int runReadback(const Options& options) {
    CheckReport report{"Tier 2: controller readback"};
    report.note("The mirror moves to the centre of the range at the start of this mode");

    if (!confirmMotion(options, "recentre, then hold position")) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};

    // Stay in open loop throughout, so this mode never engages the servo just to read replies
    session.settleOpenLoop();

    const int frameCount = options.frameCount.value_or(kDefaultFrameCount);
    int checksumFailures = 0;
    int transferFailures = 0;
    int framesDone = 0;
    Milliradians target = session.range().center();

    println("Exchanging {} frames, dumping the first {} in full", frameCount, kFramesToDump);
    ProgressReporter progress{"frames", static_cast<double>(frameCount), "frames"};

    for (int frameIndex = 0; frameIndex < frameCount && !interrupted(); ++frameIndex) {
        // Echo the last readback so that after the initial recentring nothing further moves
        if (!ensureWithinRange(session.range(), target, "readback echo")) {
            return EXIT_FAILURE;
        }

        // Use exchange rather than moveTo, since this mode reports on the raw reply bytes
        const Frame command = buildFrame(Servo::Open, target);
        const auto reply = session.controller().exchange(command);
        if (!reply) {
            ++transferFailures;
            continue;
        }

        const Reply decoded = decode(*reply);
        if (!decoded.checksumValid) {
            ++checksumFailures;
        }

        if (frameIndex < kFramesToDump) {
            println("  frame {}", frameIndex);
            println("    sent: {}", toHex(command));
            println("    recv: {}", toHex(*reply));
            println("    reply position: ({}, {}) mrad, checksum {}", decoded.position.x,
                    decoded.position.y, decoded.checksumValid ? "valid" : "INVALID");
            reportSuspiciousReply(report, command, *reply);
        }

        // Track the readback only while it stays sane, so one bad frame cannot walk the target away
        if (session.range().contains(decoded.position)) {
            target = decoded.position;
        }

        ++framesDone;
        progress.update(framesDone, std::format("checksum failures {}", checksumFailures));
    }
    progress.finish(framesDone, std::format("checksum failures {}", checksumFailures));
    session.park();

    report.check(transferFailures == 0,
                 std::format("every transfer completed ({} failed)", transferFailures));
    report.check(checksumFailures == 0,
                 std::format("every reply checksum validated ({} failed)", checksumFailures));
    if (checksumFailures > 0) {
        report.note("If every reply fails, the controller likely uses a different convention for its "
                    "reply framing. That is a finding, not a bug in this tool.");
    }

    return report.finish();
}

int runBadChecksum(const Options& options) {
    CheckReport report{"Tier 2: deliberate checksum corruption"};

    if (!options.allowErrorState) {
        eprintln("This mode may leave the controller in an error state needing a manual clear.");
        eprintln("Re-run with --allow-error-state to confirm.");
        return EXIT_FAILURE;
    }
    if (!confirmMotion(options, "recentre, then send one frame carrying a bad checksum")) {
        return EXIT_FAILURE;
    }

    MotionSession session{options};
    session.settleOpenLoop();

    const Frame good = buildFrame(Servo::Open, session.range().center());
    Frame bad = good;
    bad.back() ^= kStuckHigh;

    const auto before = session.controller().exchange(good);
    const auto rejected = session.controller().exchange(bad);
    const auto after = session.controller().exchange(good);

    println("  good frame   sent: {}", toHex(good));
    println("               recv: {}", showReply(before));
    println("  bad frame    sent: {}", toHex(bad));
    println("               recv: {}", showReply(rejected));
    println("  good frame   sent: {}", toHex(good));
    println("               recv: {}", showReply(after));
    session.park();

    report.check(before && rejected && after, "all three transfers completed at transport level");

    const bool replyDiffers = rejected != before;
    report.check(replyDiffers, "the reply to a corrupted frame differs from the reply to a good one");
    if (!replyDiffers) {
        report.warn("The controller gives no distinguishable response to a bad checksum, so a "
                    "caller cannot detect a rejected command from the reply alone.");
    }
    report.check(after.has_value(), "the controller still answers after receiving a corrupted frame");

    return report.finish();
}

}  // namespace e727::spi_check
