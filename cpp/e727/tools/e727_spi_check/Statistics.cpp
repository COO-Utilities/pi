/**
 * @file Statistics.cpp
 * @brief Timing distributions, histograms and line fitting for e727_spi_check results
 */

#include "Statistics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <functional>
#include <numeric>
#include <string>

#include "Report.hpp"

namespace e727::spi_check {

namespace {

// Multiples of the median used to bucket slow transfers, chosen to span ordinary USB hiccups
constexpr std::array<double, 4> kOutlierMultiples{2.0, 5.0, 10.0, 100.0};

constexpr double kOutlierSigmas = 3.0;

// Worst individual transfers listed by index, since USB scheduling stalls often repeat
constexpr std::size_t kWorstSamplesToList = 5;

constexpr std::size_t kHistogramBuckets = 12;
constexpr std::size_t kHistogramBarWidth = 50;
constexpr int kHistogramLabelWidth = 22;

constexpr double kMedian = 0.50;
constexpr double kP90 = 0.90;
constexpr double kP99 = 0.99;
constexpr double kP999 = 0.999;
constexpr double kPercent = 100.0;

[[nodiscard]] double shareOf(std::size_t count, std::size_t total) noexcept {
    return kPercent * static_cast<double>(count) / static_cast<double>(total);
}

[[nodiscard]] std::size_t countAbove(std::span<const double> samples, double threshold) {
    return static_cast<std::size_t>(
        std::ranges::count_if(samples, [threshold](double sample) { return sample > threshold; }));
}

}  // namespace

double percentileOf(std::span<const double> sorted, double fraction) noexcept {
    if (sorted.empty()) {
        return 0.0;
    }
    const auto index = static_cast<std::size_t>(fraction * static_cast<double>(sorted.size() - 1));
    return sorted[index];
}

TimingSummary summarize(std::vector<double> samples) {
    if (samples.empty()) {
        return {};
    }

    std::ranges::sort(samples);
    const auto count = static_cast<double>(samples.size());
    const double mean = std::accumulate(samples.begin(), samples.end(), 0.0) / count;
    const double squaredDeviations = std::transform_reduce(
        samples.begin(), samples.end(), 0.0, std::plus<>{},
        [mean](double sample) { return (sample - mean) * (sample - mean); });

    return {
        .count = samples.size(),
        .min = samples.front(),
        .max = samples.back(),
        .mean = mean,
        .median = percentileOf(samples, kMedian),
        .p90 = percentileOf(samples, kP90),
        .p99 = percentileOf(samples, kP99),
        .p999 = percentileOf(samples, kP999),
        .standardDeviation = std::sqrt(squaredDeviations / count),
    };
}

void printTimingSummary(std::string_view label, const TimingSummary& summary) {
    if (summary.count == 0) {
        println("  {}: no samples recorded", label);
        return;
    }

    println("  {} ({} samples, microseconds)", label, summary.count);
    println("    min    = {}", summary.min);
    println("    mean   = {}", summary.mean);
    println("    median = {}", summary.median);
    println("    p90    = {}", summary.p90);
    println("    p99    = {}", summary.p99);
    println("    p99.9  = {}", summary.p999);
    println("    max    = {}", summary.max);
    println("    stddev = {}", summary.standardDeviation);
}

void reportOutliers(std::span<const double> samples, const TimingSummary& summary) {
    if (samples.empty()) {
        return;
    }

    println("  outliers");
    for (const double multiple : kOutlierMultiples) {
        const double threshold = summary.median * multiple;
        const std::size_t count = countAbove(samples, threshold);
        println("    > {}x median ({} us): {} ({:.3f}%)", multiple, threshold, count,
                shareOf(count, samples.size()));
    }

    const double sigmaThreshold = summary.mean + (kOutlierSigmas * summary.standardDeviation);
    const std::size_t beyondSigma = countAbove(samples, sigmaThreshold);
    println("    > mean+3sigma ({} us): {} ({:.3f}%)", sigmaThreshold, beyondSigma,
            shareOf(beyondSigma, samples.size()));

    // Report indices as well as values, so a periodic stall shows up as evenly spaced frames
    std::vector<std::size_t> indices(samples.size());
    std::iota(indices.begin(), indices.end(), std::size_t{0});
    const std::size_t listed = std::min(kWorstSamplesToList, indices.size());
    std::ranges::partial_sort(indices, indices.begin() + static_cast<std::ptrdiff_t>(listed),
                              std::ranges::greater{},
                              [samples](std::size_t index) { return samples[index]; });

    std::string worst = "    worst transfers (frame index: duration):";
    for (const std::size_t index : std::span{indices}.first(listed)) {
        worst += std::format("  {}: {} us", index, samples[index]);
    }
    println("{}", worst);
}

void reportHistogram(std::span<const double> samples, const TimingSummary& summary) {
    if (samples.empty()) {
        return;
    }

    // Span up to p99 so that a single multi-millisecond stall cannot flatten every other bucket
    const double lower = summary.min;
    const double upper = summary.p99 > lower ? summary.p99 : lower + 1.0;
    const double width = (upper - lower) / static_cast<double>(kHistogramBuckets);

    std::vector<std::size_t> counts(kHistogramBuckets + 1, 0);
    for (const double sample : samples) {
        if (sample > upper) {
            ++counts.back();
            continue;
        }
        const auto bucket = static_cast<std::size_t>(std::max(0.0, (sample - lower) / width));
        ++counts[std::min(bucket, kHistogramBuckets - 1)];
    }

    const std::size_t busiest = std::ranges::max(counts);
    if (busiest == 0) {
        return;
    }

    println("  transfer duration histogram (microseconds)");
    for (std::size_t bucket = 0; bucket < counts.size(); ++bucket) {
        const bool isOverflow = bucket == kHistogramBuckets;
        const std::string range =
            isOverflow ? std::format(">{}", upper)
                       : std::format("{} to {}", lower + (width * static_cast<double>(bucket)),
                                     lower + (width * static_cast<double>(bucket + 1)));

        // Round up so a bucket holding a single sample still draws a mark
        const std::size_t bar = (counts[bucket] * kHistogramBarWidth + busiest - 1) / busiest;
        println("    {:>{}} | {} {}", range, kHistogramLabelWidth, std::string(bar, '#'),
                counts[bucket]);
    }
}

LinearFit fitLine(std::span<const ScalePoint> points) noexcept {
    if (points.size() < 2) {
        return {};
    }

    const auto count = static_cast<double>(points.size());
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumXX = 0.0;
    for (const ScalePoint& point : points) {
        sumX += point.commanded;
        sumY += point.readback;
        sumXY += point.commanded * point.readback;
        sumXX += point.commanded * point.commanded;
    }

    const double denominator = (count * sumXX) - (sumX * sumX);
    if (denominator == 0.0) {
        return {};
    }

    LinearFit fit;
    fit.slope = ((count * sumXY) - (sumX * sumY)) / denominator;
    fit.intercept = (sumY - (fit.slope * sumX)) / count;
    for (const ScalePoint& point : points) {
        const double predicted = (fit.slope * point.commanded) + fit.intercept;
        fit.maxResidual = std::max(fit.maxResidual, std::abs(point.readback - predicted));
    }
    return fit;
}

}  // namespace e727::spi_check
