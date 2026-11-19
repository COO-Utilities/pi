/**
 * @file Options.cpp
 * @brief Command line options for e727_spi_check
 */

#include "Options.hpp"

#include <charconv>
#include <cmath>
#include <concepts>
#include <format>
#include <system_error>
#include <utility>

namespace e727::spi_check {

namespace {

using ParseResult = std::expected<void, std::string>;

constexpr int kHexBase = 16;
constexpr char kRangeSeparator = ':';

template <typename Number>
[[nodiscard]] std::expected<Number, std::string> parseNumber(std::string_view flag,
                                                             std::string_view text,
                                                             int base = 10) {
    Number value{};
    const char* const end = text.data() + text.size();
    std::from_chars_result result{};
    if constexpr (std::floating_point<Number>) {
        result = std::from_chars(text.data(), end, value);
    } else {
        result = std::from_chars(text.data(), end, value, base);
    }

    if (result.ec != std::errc{} || result.ptr != end) {
        return std::unexpected{std::format("Invalid value for {}: '{}'", flag, text)};
    }
    return value;
}

template <typename Number>
[[nodiscard]] std::expected<Number, std::string> parsePositive(std::string_view flag,
                                                               std::string_view text) {
    return parseNumber<Number>(flag, text).and_then(
        [flag](Number value) -> std::expected<Number, std::string> {
            bool usable = value > 0;
            if constexpr (std::floating_point<Number>) {
                usable = usable && std::isfinite(value);
            }
            if (!usable) {
                return std::unexpected{std::format("{} must be a positive number", flag)};
            }
            return value;
        });
}

[[nodiscard]] std::expected<WireAxis, std::string> parseAxis(std::string_view text) {
    if (text == "x") {
        return WireAxis::X;
    }
    if (text == "y") {
        return WireAxis::Y;
    }
    return std::unexpected{std::format("--axis must be x or y, not '{}'", text)};
}

[[nodiscard]] std::expected<TravelRange, std::string> parseRange(std::string_view text) {
    const std::size_t separator = text.find(kRangeSeparator);
    if (separator == std::string_view::npos) {
        return std::unexpected{std::format("--range must look like <min>:<max>, not '{}'", text)};
    }

    const auto min = parseNumber<double>("--range", text.substr(0, separator));
    const auto max = parseNumber<double>("--range", text.substr(separator + 1));
    if (!min || !max) {
        return std::unexpected{min ? max.error() : min.error()};
    }

    const TravelRange range{.min = *min, .max = *max};
    if (!range.isValid()) {
        return std::unexpected{std::format("--range {} needs finite limits with min below max", text)};
    }
    return range;
}

// Build a callback that stores a parsed value, for use with std::expected::transform
template <typename Field>
[[nodiscard]] auto storeIn(Field& field) {
    return [&field](auto value) { field = std::move(value); };
}

[[nodiscard]] ParseResult applyFlag(Options& options, std::string_view flag, std::string_view value) {
    if (flag == "--device") {
        return parseNumber<int>(flag, value).transform(storeIn(options.deviceIndex));
    }
    if (flag == "--pid") {
        return parseNumber<std::uint32_t>(flag, value, kHexBase).transform(storeIn(options.usbProductId));
    }
    if (flag == "--vid") {
        return parseNumber<std::uint32_t>(flag, value, kHexBase).transform(storeIn(options.usbVendorId));
    }
    if (flag == "--axis") {
        return parseAxis(value).transform(storeIn(options.axis));
    }
    if (flag == "--range") {
        return parseRange(value).transform(storeIn(options.range));
    }
    if (flag == "--amplitude") {
        return parsePositive<double>(flag, value).transform(storeIn(options.amplitude));
    }
    if (flag == "--rate") {
        return parsePositive<double>(flag, value).transform(storeIn(options.rateHz));
    }
    if (flag == "--duration") {
        return parsePositive<double>(flag, value).transform(storeIn(options.durationSeconds));
    }
    if (flag == "--frames") {
        return parsePositive<int>(flag, value).transform(storeIn(options.frameCount));
    }
    if (flag == "--csv") {
        options.csvPath = value;
        return {};
    }
    return std::unexpected{std::format("Unknown flag: {}", flag)};
}

}  // namespace

std::expected<Options, std::string> parseOptions(std::span<const std::string_view> arguments) {
    if (arguments.empty()) {
        return std::unexpected{std::string{"No mode given"}};
    }

    Options options{.mode = std::string{arguments.front()}};

    for (std::size_t i = 1; i < arguments.size(); ++i) {
        const std::string_view flag = arguments[i];

        if (flag == "--yes") {
            options.assumeYes = true;
            continue;
        }
        if (flag == "--allow-error-state") {
            options.allowErrorState = true;
            continue;
        }

        // Every remaining flag takes a value
        if (i + 1 >= arguments.size()) {
            return std::unexpected{std::format("Missing value for {}", flag)};
        }
        if (ParseResult applied = applyFlag(options, flag, arguments[++i]); !applied) {
            return std::unexpected{std::move(applied.error())};
        }
    }

    return options;
}

}  // namespace e727::spi_check
