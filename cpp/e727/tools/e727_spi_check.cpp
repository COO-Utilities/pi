/**
 * @file e727_spi_check.cpp
 * @brief Bring-up diagnostic for the PI E-727 over an FT4222 USB-to-SPI bridge
 *
 * Run the modes in order, since each assumes the one before it passed:
 *   probe     find and open the FT4222, no controller needed
 *   readback  exchange open-loop frames at the centre and validate the replies
 *   move      step one axis across the range and check the readback follows
 *   speed     send frames back to back and report rate, transfer times and errors
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <expected>
#include <format>
#include <iostream>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "E727SpiLink.hpp"

namespace e727::spi_check {

namespace {

using Clock = std::chrono::steady_clock;
using Microseconds = std::chrono::duration<double, std::micro>;

constexpr DWORD kFtdiVendorId = 0x0403;
constexpr double kDefaultDurationSeconds = 10.0;

// A reply reports the state from before its own frame, so every settle step is sent twice
constexpr int kSettleRepeats = 2;
constexpr auto kSettleDelay = std::chrono::milliseconds(330);

constexpr int kReadbackFrames = 32;
constexpr int kFramesToDump = 4;

// Sweep ends stay this fraction of the span away from the travel limits
constexpr double kSweepMarginFraction = 0.04;
constexpr std::size_t kSweepPoints = 11;
constexpr int kSamplesPerPoint = 8;
constexpr double kSlopeTolerance = 0.05;

constexpr double kTrackingLoopHz = 500.0;

template <typename... Args>
void println(std::format_string<Args...> format, Args&&... args) {
    std::cout << std::format(format, std::forward<Args>(args)...) << '\n';
}

// Options ---------------------------------------------------------------------------------------

enum class Axis : std::uint8_t { X, Y };

[[nodiscard]] constexpr std::string_view nameOf(Axis axis) noexcept {
    return axis == Axis::X ? "x" : "y";
}

struct Options {
    std::string_view mode;
    int deviceIndex = 0;
    std::optional<DWORD> productId;
    Axis axis = Axis::X;
    TravelRange range = kDefaultRange;
    double durationSeconds = kDefaultDurationSeconds;
    bool assumeYes = false;
};

template <typename Number>
[[nodiscard]] std::optional<Number> parseNumber(std::string_view text, int base = 10) {
    Number value{};
    const char* const end = text.data() + text.size();
    std::from_chars_result result{};
    if constexpr (std::is_floating_point_v<Number>) {
        result = std::from_chars(text.data(), end, value);
    } else {
        result = std::from_chars(text.data(), end, value, base);
    }
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::nullopt;
    }
    return value;
}

[[nodiscard]] std::optional<TravelRange> parseRange(std::string_view text) {
    const std::size_t separator = text.find(':');
    if (separator == std::string_view::npos) {
        return std::nullopt;
    }
    const auto min = parseNumber<double>(text.substr(0, separator));
    const auto max = parseNumber<double>(text.substr(separator + 1));
    if (!min || !max || !TravelRange{.min = *min, .max = *max}.isValid()) {
        return std::nullopt;
    }
    return TravelRange{.min = *min, .max = *max};
}

template <typename Target, typename Value>
[[nodiscard]] bool assign(Target& target, const std::optional<Value>& value) {
    if (value) {
        target = *value;
    }
    return value.has_value();
}

[[nodiscard]] bool applyFlag(Options& options, std::string_view flag, std::string_view value) {
    constexpr int kHexBase = 16;
    if (flag == "--device") {
        return assign(options.deviceIndex, parseNumber<int>(value));
    }
    if (flag == "--pid") {
        return assign(options.productId, parseNumber<DWORD>(value, kHexBase));
    }
    if (flag == "--range") {
        return assign(options.range, parseRange(value));
    }
    if (flag == "--duration") {
        const auto seconds = parseNumber<double>(value);
        return seconds && *seconds > 0.0 && assign(options.durationSeconds, seconds);
    }
    if (flag == "--axis" && (value == "x" || value == "y")) {
        options.axis = value == "x" ? Axis::X : Axis::Y;
        return true;
    }
    return false;
}

[[nodiscard]] std::expected<Options, std::string> parseOptions(
    std::span<const std::string_view> arguments) {
    Options options;
    options.mode = arguments.front();
    for (std::size_t i = 1; i < arguments.size(); ++i) {
        const std::string_view flag = arguments[i];
        if (flag == "--yes") {
            options.assumeYes = true;
        } else if (i + 1 == arguments.size()) {
            return std::unexpected{std::format("{} needs a value", flag)};
        } else if (const std::string_view value = arguments[++i];
                   !applyFlag(options, flag, value)) {
            return std::unexpected{std::format("unknown option or bad value: {} {}", flag, value)};
        }
    }
    return options;
}

// Reporting and safety --------------------------------------------------------------------------

/// Print each check as it runs and turn the tally into an exit code
class Checks {
public:
    void expect(bool passed, std::string_view description) {
        println("  [{}] {}", passed ? "PASS" : "FAIL", description);
        failures_ += passed ? 0 : 1;
    }

    [[nodiscard]] int finish() const {
        if (failures_ == 0) {
            println("All checks passed");
            return EXIT_SUCCESS;
        }
        println("{} check(s) failed", failures_);
        return EXIT_FAILURE;
    }

private:
    int failures_ = 0;
};

// Global because a signal handler can reach nothing else; device I/O is not async-signal-safe
std::atomic<bool> interruptRequested{false};

void handleInterrupt(int /*signal*/) { interruptRequested.store(true); }

[[nodiscard]] bool interrupted() noexcept { return interruptRequested.load(); }

[[nodiscard]] bool confirmMotion(const Options& options, std::string_view description) {
    println("!! This mode moves the mirror: {}", description);
    println("!! Travel range [{}, {}] mrad; targets outside it are refused", options.range.min,
            options.range.max);
    if (options.assumeYes) {
        return true;
    }
    std::cout << "Type 'yes' to proceed: " << std::flush;
    std::string response;
    if (std::getline(std::cin, response) && response == "yes") {
        return true;
    }
    println("\nAborted");
    return false;
}

[[nodiscard]] SpiController openController(const Options& options) {
    println("Opening FT4222 device {}...", options.deviceIndex);
    SpiController controller{options.deviceIndex, options.range};
    // The system clock is inherited rather than set and skews every timing result, so show it
    println("Chip mode {}, system clock {}", controller.chipMode(),
            describe(controller.clockRate()));
    return controller;
}

/// Controller with its servo loops closed at the centre, parked again on destruction
class MotionSession {
public:
    explicit MotionSession(const Options& options) : controller_(openController(options)) {
        controller_.openLoop();
        settle();
        controller_.closeLoop();
        settle();
    }

    MotionSession(const MotionSession&) = delete;
    MotionSession& operator=(const MotionSession&) = delete;

    // Centre under servo control before releasing, so the mirror is never dropped from an offset
    ~MotionSession() {
        println("Parking at the centre with the loops open");
        controller_.closeLoop();
        settle();
        controller_.openLoop();
        settle();
    }

    [[nodiscard]] SpiController& controller() noexcept { return controller_; }

private:
    void settle() {
        for (int repeat = 0; repeat < kSettleRepeats; ++repeat) {
            (void)controller_.moveTo(controller_.range().center());
            std::this_thread::sleep_for(kSettleDelay);
        }
    }

    SpiController controller_;
};

// Modes -----------------------------------------------------------------------------------------

int runProbe(const Options& options) {
    const std::vector<UsbDevice> devices = listUsbDevices();
    println("{} FTDI device(s) visible to D2XX", devices.size());
    for (const auto& [index, device] : std::views::enumerate(devices)) {
        println("  [{}] type 0x{:X} \"{}\" serial \"{}\"{}", index, device.type,
                device.description, device.serial, device.isFt4222() ? "  <- FT4222" : "");
    }

    Checks checks;
    const bool found = std::ranges::any_of(devices, &UsbDevice::isFt4222);
    checks.expect(found, "an FT4222 is attached");
    if (!found) {
        println("A board with a custom USB product id stays invisible until named: find it with");
        println("lsusb (vendor 0403) and re-run with --pid <hex>");
        return checks.finish();
    }
    const SpiController controller = openController(options);
    checks.expect(true, "opened and configured as SPI master");
    return checks.finish();
}

// Patterns that mean the controller is not driving MISO at all
[[nodiscard]] std::string_view suspectReply(const Frame& sent, const Frame& reply) {
    if (reply == sent) {
        return "reply equals the command: is MISO still jumpered to MOSI?";
    }
    if (std::ranges::all_of(reply, [&reply](std::uint8_t byte) { return byte == reply.front(); })) {
        return "every reply byte is the same: is the controller powered and MISO connected?";
    }
    return {};
}

int runReadback(const Options& options) {
    if (!confirmMotion(options, "recentre in open loop and hold")) {
        return EXIT_FAILURE;
    }
    // A new controller holds an open-loop frame at the centre, so this mode never closes the loop
    SpiController controller = openController(options);
    const Frame command = controller.lastCommand();

    int transferFailures = 0;
    int checksumFailures = 0;
    for (int frame = 0; frame < kReadbackFrames && !interrupted(); ++frame) {
        const auto reply = controller.exchange(command);
        if (!reply) {
            ++transferFailures;
            continue;
        }
        const Reply decoded = decode(*reply);
        checksumFailures += decoded.checksumValid ? 0 : 1;
        if (frame < kFramesToDump) {
            println("  sent {}", toHex(command));
            println("  recv {}  ({}, {}) mrad, checksum {}", toHex(*reply), decoded.position.x,
                    decoded.position.y, decoded.checksumValid ? "valid" : "INVALID");
            if (const std::string_view suspect = suspectReply(command, *reply); !suspect.empty()) {
                println("  !! {}", suspect);
            }
        }
    }

    Checks checks;
    checks.expect(transferFailures == 0,
                  std::format("every transfer completed ({} failed)", transferFailures));
    checks.expect(checksumFailures == 0,
                  std::format("every reply checksum is valid ({} invalid)", checksumFailures));
    return checks.finish();
}

struct SweepPoint {
    double commanded = 0.0;
    double observed = 0.0;
};

// Mean of the valid replies while holding `target`, if any arrived
[[nodiscard]] std::optional<Milliradians> meanReadback(SpiController& controller,
                                                       Milliradians target) {
    Milliradians total{};
    int samples = 0;
    for (int sample = 0; sample < kSamplesPerPoint; ++sample) {
        if (const auto reply = controller.moveTo(target); reply && reply->checksumValid) {
            total = {.x = total.x + reply->position.x, .y = total.y + reply->position.y};
            ++samples;
        }
    }
    if (samples == 0) {
        return std::nullopt;
    }
    return Milliradians{.x = total.x / samples, .y = total.y / samples};
}

[[nodiscard]] std::vector<SweepPoint> sweepAxis(const Options& options) {
    const auto along = [&options](Milliradians position) {
        return options.axis == Axis::X ? position.x : position.y;
    };
    const auto across = [&options](Milliradians position) {
        return options.axis == Axis::X ? position.y : position.x;
    };
    const double margin = options.range.span() * kSweepMarginFraction;
    const double step = (options.range.span() - (2.0 * margin)) / (kSweepPoints - 1);

    MotionSession session{options};
    std::vector<SweepPoint> points;
    println("  {:>10} {:>10} {:>10} {:>10}", "command", "readback", "residual", "other axis");
    for (std::size_t point = 0; point < kSweepPoints && !interrupted(); ++point) {
        const double commanded = options.range.min + margin + (step * static_cast<double>(point));
        Milliradians target = options.range.center();
        (options.axis == Axis::X ? target.x : target.y) = commanded;

        (void)session.controller().moveTo(target);
        std::this_thread::sleep_for(kSettleDelay);
        if (const auto readback = meanReadback(session.controller(), target)) {
            const double observed = along(*readback);
            println("  {:>10.4f} {:>10.4f} {:>10.4f} {:>10.4f}", commanded, observed,
                    observed - commanded, across(*readback));
            points.push_back({.commanded = commanded, .observed = observed});
        }
    }
    return points;
}

// Least-squares slope of observed against commanded, NaN with fewer than two points
[[nodiscard]] double fitSlope(std::span<const SweepPoint> points) {
    if (points.size() < 2) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto count = static_cast<double>(points.size());
    double meanCommanded = 0.0;
    double meanObserved = 0.0;
    for (const SweepPoint& point : points) {
        meanCommanded += point.commanded / count;
        meanObserved += point.observed / count;
    }
    double covariance = 0.0;
    double variance = 0.0;
    for (const SweepPoint& point : points) {
        const double offset = point.commanded - meanCommanded;
        covariance += offset * (point.observed - meanObserved);
        variance += offset * offset;
    }
    return covariance / variance;
}

int runMove(const Options& options) {
    if (!confirmMotion(options, std::format("step wire axis {} across the range in {} points",
                                            nameOf(options.axis), kSweepPoints))) {
        return EXIT_FAILURE;
    }
    const std::vector<SweepPoint> points = sweepAxis(options);
    const double slope = fitSlope(points);

    Checks checks;
    checks.expect(points.size() == kSweepPoints, "every point returned a valid readback");
    checks.expect(std::abs(slope - 1.0) < kSlopeTolerance,
                  std::format("readback follows command with slope {:.4f}", slope));
    println("A slope far from 1 means the controller is not configured for [{}, {}] mrad.",
            options.range.min, options.range.max);
    println("The physical axis that moved on the camera is wire axis {}.", nameOf(options.axis));
    return checks.finish();
}

struct SpeedRun {
    std::vector<float> transferMicroseconds;
    int transferFailures = 0;
    int checksumFailures = 0;
    double seconds = 0.0;
};

[[nodiscard]] SpeedRun sendBackToBack(const Options& options) {
    MotionSession session{options};
    SpeedRun run;
    const auto start = Clock::now();
    const auto deadline = start + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double>(options.durationSeconds));

    while (Clock::now() < deadline && !interrupted()) {
        const auto before = Clock::now();
        const auto reply = session.controller().position();
        run.transferMicroseconds.push_back(
            static_cast<float>(Microseconds(Clock::now() - before).count()));
        if (!reply) {
            ++run.transferFailures;
        } else if (!reply->checksumValid) {
            ++run.checksumFailures;
        }
    }
    run.seconds = std::chrono::duration<double>(Clock::now() - start).count();
    return run;
}

// Value at `fraction` of the way through already sorted samples
[[nodiscard]] float percentile(std::span<const float> sorted, double fraction) {
    const auto index = static_cast<std::size_t>(fraction * static_cast<double>(sorted.size() - 1));
    return sorted[index];
}

int runSpeed(const Options& options) {
    if (!confirmMotion(options, std::format("hold the centre for {} s", options.durationSeconds))) {
        return EXIT_FAILURE;
    }
    SpeedRun run = sendBackToBack(options);
    Checks checks;
    if (run.transferMicroseconds.empty()) {
        checks.expect(false, "at least one frame was sent");
        return checks.finish();
    }

    std::ranges::sort(run.transferMicroseconds);
    const std::span<const float> sorted = run.transferMicroseconds;
    const double rateHz = static_cast<double>(sorted.size()) / run.seconds;
    const double budgetMicroseconds =
        Microseconds(std::chrono::seconds(1)).count() / kTrackingLoopHz;
    const float p99 = percentile(sorted, 0.99);

    println("  {} frames in {:.1f} s, {:.0f} Hz", sorted.size(), run.seconds, rateHz);
    println("  transfer us: median {:.0f}, p99 {:.0f}, p99.9 {:.0f}, max {:.0f}",
            percentile(sorted, 0.5), p99, percentile(sorted, 0.999), sorted.back());

    checks.expect(run.transferFailures == 0,
                  std::format("no transfer failures ({})", run.transferFailures));
    checks.expect(run.checksumFailures == 0,
                  std::format("no checksum failures ({})", run.checksumFailures));
    checks.expect(rateHz > kTrackingLoopHz,
                  std::format("rate clears the {} Hz tracking loop", kTrackingLoopHz));
    checks.expect(p99 < budgetMicroseconds,
                  std::format("p99 transfer fits the {} us loop budget", budgetMicroseconds));
    return checks.finish();
}

// Entry point -----------------------------------------------------------------------------------

struct Mode {
    std::string_view name;
    int (*run)(const Options&);
    std::string_view summary;
};

constexpr std::array kModes{
    Mode{"probe", runProbe, "list FTDI devices, open the FT4222, show its clock"},
    Mode{"readback", runReadback, "open-loop frames at the centre, validate replies"},
    Mode{"move", runMove, "step one axis across the range, check readback follows"},
    Mode{"speed", runSpeed, "frames back to back: rate, transfer times, errors"},
};

void printUsage() {
    println("Usage: e727_spi_check <mode> [options]\n\nModes, in the order to run them:");
    for (const Mode& mode : kModes) {
        println("  {:<10}{}", mode.name, mode.summary);
    }
    println("\nOptions:");
    println("  --device <n>       FT4222 device index (default 0)");
    println("  --pid <hex>        custom USB product id, for boards with a modified EEPROM");
    println("  --axis x|y         axis for move (default x)");
    println("  --range <min:max>  travel range in mrad (default {}:{})", kDefaultRange.min,
            kDefaultRange.max);
    println("  --duration <s>     length of a speed run (default {})", kDefaultDurationSeconds);
    println("  --yes              skip the confirmation before motion");
    println("\nCtrl-C stops a run early and parks the mirror at the centre with the loops open.");
}

}  // namespace

int run(std::span<const std::string_view> arguments) {
    if (arguments.empty() || arguments.front() == "--help" || arguments.front() == "-h") {
        printUsage();
        return arguments.empty() ? EXIT_FAILURE : EXIT_SUCCESS;
    }
    const auto options = parseOptions(arguments);
    if (!options) {
        std::cerr << options.error() << '\n';
        return EXIT_FAILURE;
    }
    const auto mode = std::ranges::find(kModes, options->mode, &Mode::name);
    if (mode == kModes.end()) {
        std::cerr << std::format("Unknown mode '{}', see --help\n", options->mode);
        return EXIT_FAILURE;
    }
    // D2XX ignores a board with a custom product id until told about it, before any enumeration
    if (options->productId && !registerUsbId(kFtdiVendorId, *options->productId)) {
        std::cerr << std::format("Could not register product id {:04x}\n", *options->productId);
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, handleInterrupt);
    try {
        return mode->run(*options);
    } catch (const std::runtime_error& error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}

}  // namespace e727::spi_check

int main(int argc, char** argv) {
    const std::vector<std::string_view> arguments(argv + 1, argv + argc);
    return e727::spi_check::run(arguments);
}
