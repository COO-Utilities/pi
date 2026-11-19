/**
 * @file Session.cpp
 * @brief Hardware sequences, safety checks and interrupt handling shared by e727_spi_check modes
 */

#include "Session.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "Report.hpp"

namespace e727::spi_check {

namespace {

// Global because a signal handler can reach nothing else
std::atomic<bool> interruptRequested{false};

// Record the request only, since device I/O is not async-signal-safe
void handleInterrupt(int /*signal*/) { interruptRequested.store(true, std::memory_order_relaxed); }

constexpr auto kBusyWaitSlice = std::chrono::microseconds(50);

// A reply reports the state from before its own frame, so each handshake step is sent twice
constexpr int kHandshakeRepeats = 2;

}  // namespace

void installInterruptHandler() { std::signal(SIGINT, handleInterrupt); }

bool interrupted() noexcept { return interruptRequested.load(std::memory_order_relaxed); }

void waitUntil(std::chrono::steady_clock::time_point deadline) {
    while (std::chrono::steady_clock::now() < deadline && !interrupted()) {
        std::this_thread::sleep_for(kBusyWaitSlice);
    }
}

bool confirmMotion(const Options& options, std::string_view description) {
    println();
    println("!! This mode moves the mirror: {}", description);
    println("!! Commands outside [{}, {}] mrad are refused.", options.range.min, options.range.max);

    if (options.assumeYes) {
        println("Proceeding (--yes given)");
        return true;
    }

    std::cout << "Type 'yes' to proceed: " << std::flush;
    std::string response;
    if (!std::getline(std::cin, response) || response != "yes") {
        println("Aborted");
        return false;
    }
    return true;
}

bool ensureWithinRange(const TravelRange& range, Milliradians position, std::string_view context) {
    if (range.contains(position)) {
        return true;
    }
    eprintln("REFUSING {}: position ({}, {}) mrad falls outside [{}, {}]", context, position.x,
             position.y, range.min, range.max);
    return false;
}

bool ensureTrajectoryWithinRange(const TravelRange& range, std::span<const Milliradians> trajectory) {
    return std::ranges::all_of(trajectory, [&range](Milliradians position) {
        return ensureWithinRange(range, position, "trajectory");
    });
}

std::string_view axisLabel(WireAxis axis) noexcept {
    return axis == WireAxis::X ? "x (bytes 4-7)" : "y (bytes 8-11)";
}

double valueOn(WireAxis axis, Milliradians position) noexcept {
    return axis == WireAxis::X ? position.x : position.y;
}

Milliradians positionOn(WireAxis axis, double value, const TravelRange& range) noexcept {
    Milliradians position = range.center();
    (axis == WireAxis::X ? position.x : position.y) = value;
    return position;
}

SpiController openController(const Options& options) {
    // Announce first, since opening can block or throw
    println("Opening FT4222 device {} and configuring SPI master mode...", options.deviceIndex);
    SpiController controller{options.deviceIndex, options.range};

    // Log the clock rate every run: it is inherited rather than set, and it skews any timing result
    println("Device open: chip mode {}, clock rate {}, travel [{}, {}] mrad", controller.chipMode(),
            e727::describe(controller.clockRate()), options.range.min, options.range.max);
    return controller;
}

MotionSession::MotionSession(const Options& options) : controller_(openController(options)) {
    installInterruptHandler();
}

MotionSession::~MotionSession() { park(); }

void MotionSession::settleAt(Milliradians target) {
    for (int repeat = 0; repeat < kHandshakeRepeats; ++repeat) {
        (void)controller_.moveTo(target);
        std::this_thread::sleep_for(kSettleDelay);
    }
}

void MotionSession::settleOpenLoop() {
    const Milliradians center = range().center();
    println("Settling at the centre ({} mrad) in open loop, {} frames", center.x, kHandshakeRepeats);
    controller_.openLoop();
    settleAt(center);
}

void MotionSession::handshake(Milliradians start) {
    settleOpenLoop();
    println("Closing servo loops at ({}, {}) mrad, {} frames", start.x, start.y, kHandshakeRepeats);
    controller_.closeLoop();
    settleAt(start);
    println("Handshake complete");
}

void MotionSession::park() {
    if (parked_) {
        return;
    }
    parked_ = true;
    println("Parking mirror at the centre and opening servo loops...");

    // Centre under servo control first, then release, so the mirror is not dropped from an offset
    const Milliradians center = range().center();
    controller_.closeLoop();
    (void)controller_.moveTo(center);
    std::this_thread::sleep_for(kSettleDelay);

    controller_.openLoop();
    (void)controller_.moveTo(center);
    std::this_thread::sleep_for(kSettleDelay);

    println("Mirror parked");
}

}  // namespace e727::spi_check
