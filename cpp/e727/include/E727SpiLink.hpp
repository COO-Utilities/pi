/**
 * @file E727SpiLink.hpp
 * @brief FT4222 USB-to-SPI link to the PI E-727 fast steering mirror controller
 *
 * The protocol is full duplex: the transaction that carries a command also carries the
 * controller's reply, so moveTo() returns a position and there is no read-only frame. position()
 * re-sends the last command to take a reading without moving the target.
 *
 * Hardware requirement: an FTDI FT4222H USB-to-SPI bridge between the host and the E-727.
 * Device discovery, SPI configuration and every transfer go through FTDI's LibFT4222 and D2XX
 * (tested version in cpp/README.md), which drive only that chip, so no other USB-to-SPI adapter
 * or native SPI port works with this header. Supporting another bridge means a new link header;
 * E727Protocol.hpp carries over unchanged. Linux only.
 */

#pragma once

#include <cstdint>
#include <expected>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "E727Protocol.hpp"
#include "ftd2xx.h"
#include "libft4222.h"

namespace e727 {

enum class Error : std::uint8_t {
    OutOfRange,
    TransferFailed,
    ShortTransfer,
};

[[nodiscard]] constexpr std::string_view describe(Error error) noexcept {
    switch (error) {
        case Error::OutOfRange: return "target outside the travel range, nothing sent";
        case Error::TransferFailed: return "LibFT4222 reported a transfer error";
        case Error::ShortTransfer: return "transfer moved fewer bytes than a frame";
    }
    return "unknown error";
}

[[nodiscard]] constexpr std::string_view describe(FT4222_ClockRate rate) noexcept {
    switch (rate) {
        case SYS_CLK_60: return "60 MHz";
        case SYS_CLK_24: return "24 MHz";
        case SYS_CLK_48: return "48 MHz";
        case SYS_CLK_80: return "80 MHz";
    }
    return "unknown clock rate";
}

// SPI settings the E-727 expects --------------------------------------------------------------

inline constexpr FT4222_SPIMode kSpiMode = SPI_IO_SINGLE;
inline constexpr FT4222_SPIClock kSpiClockDivider = CLK_DIV_4;
inline constexpr FT4222_SPICPOL kSpiClockPolarity = CLK_IDLE_LOW;
inline constexpr FT4222_SPICPHA kSpiClockPhase = CLK_TRAILING;
inline constexpr std::uint8_t kSlaveSelectSs0 = 0x01;

// Device discovery ----------------------------------------------------------------------------

/// One FTDI device as reported by D2XX enumeration
struct UsbDevice {
    std::string description;
    std::string serial;
    DWORD flags = 0;
    DWORD type = 0;
    DWORD locationId = 0;

    [[nodiscard]] bool isOpen() const noexcept { return (flags & FT_FLAGS_OPENED) != 0; }
    [[nodiscard]] bool isHighSpeed() const noexcept { return (flags & FT_FLAGS_HISPEED) != 0; }

    [[nodiscard]] bool isFt4222() const noexcept {
        // Trust the reported type first: a custom EEPROM can rename the description to anything
        switch (type) {
            case FT_DEVICE_4222H_0:
            case FT_DEVICE_4222H_1_2:
            case FT_DEVICE_4222H_3:
            case FT_DEVICE_4222_PROG: return true;
            default: return description.contains("FT4222");
        }
    }
};

/**
 * Teach D2XX a non-standard USB vendor and product id.
 *
 * D2XX matches devices against a built-in id table, so a board whose EEPROM carries a custom
 * product id enumerates as nothing at all, with no error. Call before any enumeration or open;
 * lsusb shows the real id.
 */
[[nodiscard]] inline bool registerUsbId(DWORD vendorId, DWORD productId) noexcept {
    return FT_SetVIDPID(vendorId, productId) == FT_OK;
}

/// Enumerate every FTDI device D2XX can see, or none if enumeration fails
[[nodiscard]] inline std::vector<UsbDevice> listUsbDevices() {
    DWORD count = 0;
    if (FT_CreateDeviceInfoList(&count) != FT_OK) {
        return {};
    }

    std::vector<UsbDevice> devices;
    devices.reserve(count);
    for (DWORD index = 0; index < count; ++index) {
        FT_DEVICE_LIST_INFO_NODE node{};
        const FT_STATUS status =
            FT_GetDeviceInfoDetail(index, &node.Flags, &node.Type, &node.ID, &node.LocId,
                                   node.SerialNumber, node.Description, &node.ftHandle);
        if (status == FT_OK) {
            devices.push_back({.description = node.Description,
                               .serial = node.SerialNumber,
                               .flags = node.Flags,
                               .type = node.Type,
                               .locationId = node.LocId});
        }
    }
    return devices;
}

[[nodiscard]] inline std::vector<UsbDevice> listFt4222Devices() {
    std::vector<UsbDevice> devices = listUsbDevices();
    std::erase_if(devices, [](const UsbDevice& device) { return !device.isFt4222(); });
    return devices;
}

// The link ------------------------------------------------------------------------------------

namespace detail {

// Release the SPI master before closing, the order LibFT4222 requires
struct HandleCloser {
    using pointer = FT_HANDLE;

    void operator()(FT_HANDLE handle) const noexcept {
        FT4222_UnInitialize(handle);
        FT_Close(handle);
    }
};

using Handle = std::unique_ptr<void, HandleCloser>;

}  // namespace detail

/// FT4222 configured as SPI master, owning the link to one E-727
class SpiController {
public:
    /**
     * Open FT4222 device `deviceIndex` as SPI master, refusing commands outside `range`.
     *
     * Throws std::invalid_argument for an invalid range, before touching the device, and
     * std::runtime_error if the device cannot be opened or configured.
     */
    explicit SpiController(int deviceIndex = 0, TravelRange range = kDefaultRange);

    /// Confirm the controller answers with a valid checksum, by re-sending the last command
    [[nodiscard]] bool ping();

    /**
     * Command both axes and return the position reported in the same transaction.
     *
     * There is no overload taking bare doubles, so every call site names its unit. Targets
     * outside the travel range are refused before anything is sent, never clamped.
     */
    [[nodiscard]] std::expected<Reply, Error> moveTo(Milliradians target);

    [[nodiscard]] std::expected<Reply, Error> moveTo(Microradians target) {
        return moveTo(toMilliradians(target));
    }

    /// Read the position by re-sending the last command, leaving the target unchanged
    [[nodiscard]] std::expected<Reply, Error> position() { return exchange(last_).transform(decode); }

    /// Select the servo state for later commands, without sending anything now
    void closeLoop() noexcept { last_ = withServo(last_, Servo::Closed); }
    void openLoop() noexcept { last_ = withServo(last_, Servo::Open); }
    [[nodiscard]] bool isClosedLoop() const noexcept { return servoOf(last_) == Servo::Closed; }

    /// Send one raw frame unchecked and return the raw reply, for diagnostics that must see the wire
    [[nodiscard]] std::expected<Frame, Error> exchange(const Frame& command);

    [[nodiscard]] const TravelRange& range() const noexcept { return range_; }
    [[nodiscard]] const Frame& lastCommand() const noexcept { return last_; }
    [[nodiscard]] FT4222_ClockRate clockRate() const noexcept { return clockRate_; }
    [[nodiscard]] std::uint8_t chipMode() const noexcept { return chipMode_; }

private:
    [[nodiscard]] static TravelRange requireValid(TravelRange range);
    void configure();

    TravelRange range_;
    detail::Handle handle_;
    FT4222_ClockRate clockRate_ = SYS_CLK_60;
    std::uint8_t chipMode_ = 0;
    // Servo state lives only in this frame, so position() and ping() repeat it exactly
    Frame last_;
};

inline SpiController::SpiController(int deviceIndex, TravelRange range)
    : range_(requireValid(range)), last_(buildFrame(Servo::Open, range_.center())) {
    FT_HANDLE raw = nullptr;
    if (const FT_STATUS status = FT_Open(deviceIndex, &raw); status != FT_OK) {
        throw std::runtime_error(
            std::format("FT_Open failed for device {} (status {})", deviceIndex, status));
    }
    handle_.reset(raw);
    configure();
}

inline TravelRange SpiController::requireValid(TravelRange range) {
    if (!range.isValid()) {
        throw std::invalid_argument(std::format(
            "Invalid travel range [{}, {}]: limits must be finite with min below max", range.min,
            range.max));
    }
    return range;
}

inline void SpiController::configure() {
    // Chip mode is informational, so a failed read is no reason to refuse the link
    if (std::uint8_t mode = 0; FT4222_GetChipMode(handle_.get(), &mode) == FT4222_OK) {
        chipMode_ = mode;
    }

    if (FT4222_GetClock(handle_.get(), &clockRate_) != FT4222_OK) {
        throw std::runtime_error("FT4222_GetClock failed");
    }

    /*
     * Read the system clock but never set it. Setting it emits a spurious pulse on SCK, CS and
     * MOSI, and the only value that avoids the pulse (CLK_NONE) is unsupported and keeps whatever
     * rate was last used. Callers log clockRate() so drift across power cycles stays visible
     */
    const FT4222_STATUS status =
        FT4222_SPIMaster_Init(handle_.get(), kSpiMode, kSpiClockDivider, kSpiClockPolarity,
                              kSpiClockPhase, kSlaveSelectSs0);
    if (status != FT4222_OK) {
        throw std::runtime_error(
            std::format("FT4222_SPIMaster_Init failed (status {})", std::to_underlying(status)));
    }
}

inline bool SpiController::ping() {
    const auto reply = position();
    return reply && reply->checksumValid;
}

inline std::expected<Reply, Error> SpiController::moveTo(Milliradians target) {
    if (!range_.contains(target)) {
        return std::unexpected{Error::OutOfRange};
    }
    last_ = buildFrame(servoOf(last_), target);
    return exchange(last_).transform(decode);
}

inline std::expected<Frame, Error> SpiController::exchange(const Frame& command) {
    // LibFT4222 takes a mutable write buffer, so pass a copy rather than casting away const
    Frame outgoing = command;
    Frame incoming{};
    uint16 transferred = 0;
    constexpr BOOL kReleaseChipSelect = TRUE;

    const FT4222_STATUS status = FT4222_SPIMaster_SingleReadWrite(
        handle_.get(), incoming.data(), outgoing.data(), static_cast<uint16>(kFrameSize),
        &transferred, kReleaseChipSelect);

    if (status != FT4222_OK) {
        return std::unexpected{Error::TransferFailed};
    }
    if (transferred != kFrameSize) {
        return std::unexpected{Error::ShortTransfer};
    }
    return incoming;
}

}  // namespace e727
