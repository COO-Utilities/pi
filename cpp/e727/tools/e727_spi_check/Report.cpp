/**
 * @file Report.cpp
 * @brief Console output and pass/fail reporting shared by every e727_spi_check mode
 */

#include "Report.hpp"

#include <cstdlib>

namespace e727::spi_check {

namespace {

// Shortest gap between progress lines, so a long run stays visible without flooding a log
constexpr auto kProgressInterval = std::chrono::seconds(1);

constexpr double kPercent = 100.0;

}  // namespace

CheckReport::CheckReport(std::string title) : title_(std::move(title)) {
    println("=== {} ===", title_);
}

void CheckReport::check(bool passed, std::string_view description) {
    println("  [{}] {}", passed ? "PASS" : "FAIL", description);
    ++(passed ? passed_ : failed_);
}

void CheckReport::note(std::string_view message) const { println("   ..... {}", message); }

void CheckReport::warn(std::string_view message) const { println("  [WARN] {}", message); }

int CheckReport::finish() const {
    println("--- {}: {} passed, {} failed ---", title_, passed_, failed_);
    return failed_ == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

ProgressReporter::ProgressReporter(std::string label, double total, std::string unit)
    : label_(std::move(label)), unit_(std::move(unit)), total_(total) {
    println("  {}: starting, {} {} to go", label_, total_, unit_);
}

bool ProgressReporter::update(double completed, std::string_view detail) {
    if (std::chrono::steady_clock::now() - lastEmit_ < kProgressInterval) {
        return false;
    }
    emit(completed, detail);
    return true;
}

void ProgressReporter::finish(double completed, std::string_view detail) { emit(completed, detail); }

void ProgressReporter::emit(double completed, std::string_view detail) {
    lastEmit_ = std::chrono::steady_clock::now();

    std::string line = std::format("  {}: {} / {} {}", label_, completed, total_, unit_);
    if (total_ > 0.0) {
        line += std::format(" ({:.0f}%)", kPercent * completed / total_);
    }
    if (!detail.empty()) {
        line += std::format("  {}", detail);
    }
    println("{}", line);
}

}  // namespace e727::spi_check
