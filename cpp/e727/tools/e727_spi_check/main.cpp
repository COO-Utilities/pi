/**
 * @file main.cpp
 * @brief e727_spi_check: tiered diagnostics for the FT4222 SPI link to the PI E-727 controller
 *
 * Answers whether the FT4222 link is up and whether the controller interprets our frames the way
 * we think it does. Deliberately free of the tracking loop's dependencies (shared memory, camera,
 * control law), so it still builds and runs when those are broken.
 *
 * Modes are grouped into tiers of escalating hardware involvement, and each tier gates the next:
 * a conclusion about the axis mapping drawn while the SPI clock is misconfigured is worse than no
 * conclusion at all.
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

#include "Checks.hpp"
#include "E727SpiLink.hpp"
#include "Options.hpp"
#include "Report.hpp"

namespace e727::spi_check {

namespace {

/// Escalating level of hardware involvement, each gating the next
enum class Tier : std::uint8_t {
    ChipOnly,
    ControllerQuiet,
    ControllerMotion,
};

[[nodiscard]] constexpr std::string_view describe(Tier tier) noexcept {
    switch (tier) {
        case Tier::ChipOnly: return "tier 1, FT4222 only";
        case Tier::ControllerQuiet: return "tier 2, controller, minimal motion";
        case Tier::ControllerMotion: return "tier 3, controller, mirror moves";
    }
    return "unknown tier";
}

/// One selectable diagnostic mode
struct Mode {
    std::string_view name;
    Tier tier;
    int (*run)(const Options&);
    std::string_view summary;
};

constexpr std::array kModes{
    Mode{"probe", Tier::ChipOnly, runProbe, "enumerate devices, report chip mode and clock rate"},
    Mode{"loopback", Tier::ChipOnly, runLoopback, "verify the signal path with MOSI jumpered to MISO"},
    Mode{"readback", Tier::ControllerQuiet, runReadback, "exchange frames and validate the replies"},
    Mode{"badcrc", Tier::ControllerQuiet, runBadChecksum, "see how the controller answers a bad checksum"},
    Mode{"axis", Tier::ControllerMotion, runAxisMap, "sweep wire axis x or y to find its physical axis"},
    Mode{"scale", Tier::ControllerMotion, runScale, "fit readback against command across the range"},
    Mode{"latency", Tier::ControllerMotion, runLatency, "count frames until a step appears in the reply"},
    Mode{"rate", Tier::ControllerMotion, runRate, "time transfers at a fixed cadence"},
    Mode{"speed", Tier::ControllerMotion, runSpeed, "unpaced burst: rate, jitter, outliers, errors"},
    Mode{"soak", Tier::ControllerMotion, runSoak, "count link errors over a long hold"},
};

constexpr int kModeNameWidth = 10;
constexpr int kModeSummaryWidth = 52;

void printUsage() {
    println("Usage: e727_spi_check <mode> [options]");
    println();
    println("Tier 0 (frame layout and CRC vectors) is checked at compile time: if this tool built,");
    println("it passed. Run the modes below in order; each tier gates the next.");
    println();
    for (const Mode& mode : kModes) {
        println("  {:<{}}{:<{}}[{}]", mode.name, kModeNameWidth, mode.summary, kModeSummaryWidth,
                describe(mode.tier));
    }
    println();
    println("Options:");
    println("  --range <min>:<max>  travel limits in mrad (default {}:{})", kDefaultRange.min,
            kDefaultRange.max);
    println("  --device <n>         FT4222 device index (default 0)");
    println("  --pid <hex>          USB product id, if the board has a custom EEPROM (see lsusb)");
    println("  --vid <hex>          USB vendor id (default {:04x})", kFtdiVendorId);
    println("  --axis <x|y>         wire axis: x is bytes 4-7, y is bytes 8-11 (default x)");
    println("  --amplitude <mrad>   sweep or step size (default {}% of the range)",
            kDefaultAmplitudeFraction * 100.0);
    println("  --rate <hz>          transfer cadence (default {})", kDefaultRateHz);
    println("  --duration <s>       run length in seconds (default {})", kDefaultDurationSeconds);
    println("  --frames <n>         frames to exchange (default {}, speed mode 15000)",
            kDefaultFrameCount);
    println("  --csv <path>         write per-sample results to this file");
    println("  --yes                skip the motion confirmation prompt");
    println("  --allow-error-state  permit modes that may fault the controller");
}

[[nodiscard]] const Mode* findMode(std::string_view name) {
    const auto* const found = std::ranges::find(kModes, name, &Mode::name);
    return found == kModes.end() ? nullptr : found;
}

// Must run before any enumeration or open, so no mode can enumerate first and see an empty list
void registerCustomProductId(const Options& options) {
    if (!options.usbProductId) {
        return;
    }
    const bool registered = registerUsbId(options.usbVendorId, *options.usbProductId);
    println("Registering USB id {:04x}:{:04x} {}", options.usbVendorId, *options.usbProductId,
            registered ? "with the driver" : "FAILED");
}

[[nodiscard]] int run(std::span<const std::string_view> arguments) {
    if (!arguments.empty() && (arguments.front() == "--help" || arguments.front() == "-h")) {
        printUsage();
        return EXIT_SUCCESS;
    }

    const auto options = parseOptions(arguments);
    if (!options) {
        eprintln("{}", options.error());
        println();
        printUsage();
        return EXIT_FAILURE;
    }

    const Mode* const mode = findMode(options->mode);
    if (mode == nullptr) {
        eprintln("Unknown mode: {}", options->mode);
        println();
        printUsage();
        return EXIT_FAILURE;
    }

    registerCustomProductId(*options);

    try {
        return mode->run(*options);
    } catch (const std::runtime_error& error) {
        eprintln("Hardware error: {}", error.what());
    } catch (const std::invalid_argument& error) {
        eprintln("Invalid argument: {}", error.what());
    }
    return EXIT_FAILURE;
}

}  // namespace

}  // namespace e727::spi_check

int main(int argc, char* argv[]) {
    const std::vector<std::string_view> arguments(argv + 1, argv + argc);
    return e727::spi_check::run(arguments);
}
