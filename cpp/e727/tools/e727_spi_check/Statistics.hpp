/**
 * @file Statistics.hpp
 * @brief Timing distributions, histograms and line fitting for e727_spi_check results
 */

#pragma once

#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace e727::spi_check {

/// Distribution of a set of timing samples, in microseconds
struct TimingSummary {
    std::size_t count = 0;
    double min = 0.0;
    double max = 0.0;
    double mean = 0.0;
    double median = 0.0;
    double p90 = 0.0;
    double p99 = 0.0;
    double p999 = 0.0;
    double standardDeviation = 0.0;
};

/// Return the sample at `fraction` of the way through `sorted`, or 0 when it is empty
[[nodiscard]] double percentileOf(std::span<const double> sorted, double fraction) noexcept;

/// Summarise timing samples, taking them by value since the percentiles need them sorted
[[nodiscard]] TimingSummary summarize(std::vector<double> samples);

void printTimingSummary(std::string_view label, const TimingSummary& summary);

/// Count slow transfers against multiples of the median and list the worst by frame index
void reportOutliers(std::span<const double> samples, const TimingSummary& summary);

/// Draw a histogram spanning the bulk of the distribution, with a final overflow bucket
void reportHistogram(std::span<const double> samples, const TimingSummary& summary);

/// One commanded position paired with the mean position read back there
struct ScalePoint {
    double commanded = 0.0;
    double readback = 0.0;
};

/// Least-squares fit of readback against command
struct LinearFit {
    double slope = 0.0;
    double intercept = 0.0;
    double maxResidual = 0.0;
};

/// Fit readback against command, or return a zero fit when the points cannot define a line
[[nodiscard]] LinearFit fitLine(std::span<const ScalePoint> points) noexcept;

}  // namespace e727::spi_check
