/**
 * @file ChipChecks.cpp
 * @brief Tier 1 checks: FT4222 present, E-727 absent
 */

#include <array>
#include <format>
#include <fstream>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include "Checks.hpp"
#include "E727SpiLink.hpp"
#include "Report.hpp"
#include "Session.hpp"

namespace e727::spi_check {

namespace {

// Remembers the clock rate across runs, so power-cycle drift becomes visible
constexpr std::string_view kClockStateFile = "e727_spi_check_last_clock.txt";

[[nodiscard]] std::optional<std::string> readPreviousClockRate() {
    std::ifstream file{std::string{kClockStateFile}};
    std::string rate;
    if (!std::getline(file, rate)) {
        return std::nullopt;
    }
    return rate;
}

void recordClockRate(std::string_view rate) {
    std::ofstream file{std::string{kClockStateFile}};
    file << rate << '\n';
}

/*
 * The clock rate is inherited rather than set, so it can change across a power cycle without
 * anything in the software changing. Surface a change loudly instead of letting it silently shift
 * the SPI timing under a later measurement
 */
void reportClockDrift(CheckReport& report, std::string_view clockRate) {
    const std::optional<std::string> previous = readPreviousClockRate();
    if (!previous) {
        report.note(std::format("no previous clock rate on record, storing {}", clockRate));
    } else if (*previous != clockRate) {
        report.warn(std::format("clock rate changed since the last run: {} -> {}. Re-run any timing "
                                "measurements taken under the old rate.",
                                *previous, clockRate));
    } else {
        report.check(true, std::format("clock rate is unchanged since the last run ({})", clockRate));
    }
    recordClockRate(clockRate);
}

void printDeviceDetails(const std::vector<UsbDevice>& devices) {
    for (const auto& [index, device] : std::views::enumerate(devices)) {
        println("  Device {}:", index);
        println("    Description = {}", device.description);
        println("    Serial      = {}", device.serial);
        println("    Flags       = 0x{:X} ({}, {})", device.flags, device.isOpen() ? "open" : "closed",
                device.isHighSpeed() ? "high-speed USB" : "full-speed USB");
        println("    Type        = 0x{:X}", device.type);
        println("    Location    = 0x{:X}", device.locationId);
    }
}

[[nodiscard]] std::string differingBytes(const Frame& sent, const Frame& received) {
    std::string indices;
    for (std::size_t i = 0; i < kFrameSize; ++i) {
        if (sent[i] != received[i]) {
            indices += std::format("{} ", i);
        }
    }
    return indices;
}

}  // namespace

int runProbe(const Options& options) {
    CheckReport report{"Tier 1: device probe"};

    println("Enumerating FTDI devices...");
    const std::vector<UsbDevice> allDevices = listUsbDevices();
    println("Found {} FTDI device(s) visible to the driver", allDevices.size());
    for (const auto& [index, device] : std::views::enumerate(allDevices)) {
        println("    [{}] type 0x{:X}  \"{}\"  serial \"{}\"", index, device.type, device.description,
                device.serial);
    }

    const std::vector<UsbDevice> devices = listFt4222Devices();
    println("Found {} FT4222 device(s)", devices.size());
    report.check(!devices.empty(), "at least one FT4222 device is attached");

    if (devices.empty()) {
        // With the board plugged in, an empty list nearly always means a custom USB product id
        println();
        println("  If lsusb shows an FTDI device (vendor 0403) that is not listed above, the");
        println("  board has a custom USB product id. Re-run with that id, for example:");
        println("      e727_spi_check probe --pid faf0");
        return report.finish();
    }

    printDeviceDetails(devices);

    const SpiController controller = openController(options);
    report.check(true, "opened device and configured SPI master mode");
    reportClockDrift(report, e727::describe(controller.clockRate()));

    return report.finish();
}

int runLoopback(const Options& options) {
    CheckReport report{"Tier 1: MOSI to MISO loopback"};
    report.note("Jumper MOSI to MISO on the dev board and disconnect the controller before running");

    SpiController controller = openController(options);

    // Raw values chosen for distinct byte patterns; nothing is driven, so range does not apply
    constexpr std::array<Milliradians, 4> kPatterns{{
        {.x = 17.5, .y = 17.5},
        {.x = 2.0, .y = 33.0},
        {.x = 0.0, .y = 0.0},
        {.x = 20.125, .y = 14.875},
    }};

    for (const auto& [index, pattern] : std::views::enumerate(kPatterns)) {
        println("  pattern {} of {}: ({}, {})", index + 1, kPatterns.size(), pattern.x, pattern.y);

        const Frame command = buildFrame(Servo::Open, pattern);
        const auto reply = controller.exchange(command);
        if (!reply) {
            report.check(false, std::format("transfer of {} completed ({})", toHex(command),
                                            e727::describe(reply.error())));
            continue;
        }

        const bool identical = *reply == command;
        report.check(identical,
                     std::format("reply matches the frame sent for ({}, {})", pattern.x, pattern.y));
        if (!identical) {
            report.note("sent: " + toHex(command));
            report.note("recv: " + toHex(*reply));
            report.note("differing byte indices: " + differingBytes(command, *reply));
        }
    }

    return report.finish();
}

}  // namespace e727::spi_check
