/**
 * @file Report.hpp
 * @brief Console output and pass/fail reporting shared by every e727_spi_check mode
 */

#pragma once

#include <chrono>
#include <format>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>

namespace e727::spi_check {

// Stand-ins for std::println, which GCC 13 (the hsfei toolchain) does not ship
template <typename... Args>
void println(std::format_string<Args...> format, Args&&... args) {
    std::cout << std::format(format, std::forward<Args>(args)...) << '\n';
}

inline void println() { std::cout << '\n'; }

template <typename... Args>
void eprintln(std::format_string<Args...> format, Args&&... args) {
    std::cerr << std::format(format, std::forward<Args>(args)...) << '\n';
}

/// Collect pass/fail results so a mode reports every check rather than stopping at the first
class CheckReport {
public:
    explicit CheckReport(std::string title);

    void check(bool passed, std::string_view description);
    void note(std::string_view message) const;
    void warn(std::string_view message) const;

    /// Print the tally and return the process exit status it implies
    [[nodiscard]] int finish() const;

private:
    std::string title_;
    int passed_ = 0;
    int failed_ = 0;
};

/**
 * Emit a progress line at a bounded rate during a long run.
 *
 * Rate limited rather than printed per iteration, because these loops run at up to several kHz
 * and per-frame output would both flood the terminal and perturb the timing being measured.
 */
class ProgressReporter {
public:
    ProgressReporter(std::string label, double total, std::string unit);

    /// Return whether a line was printed, so callers can reset any per-window statistics
    bool update(double completed, std::string_view detail);

    /// Print unconditionally, for the final line of a run
    void finish(double completed, std::string_view detail);

private:
    void emit(double completed, std::string_view detail);

    std::string label_;
    std::string unit_;
    double total_ = 0.0;
    std::chrono::steady_clock::time_point lastEmit_ = std::chrono::steady_clock::now();
};

}  // namespace e727::spi_check
