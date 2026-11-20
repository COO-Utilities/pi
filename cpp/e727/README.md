# PI E-727 over FT4222 SPI

Header-only C++23 interface to the PI E-727 piezo controller that drives the HISPEC fast steering
mirror, through an FTDI FT4222H USB-to-SPI bridge, plus the `e727_spi_check` diagnostic tool.

| Path | Purpose | Needs the SDK |
|---|---|---|
| `include/E727Protocol.hpp` | Units, travel range, frame codec, CRC; usable at compile time | no |
| `include/E727SpiLink.hpp` | Device discovery and `e727::SpiController` | yes |
| `tests/E727ProtocolTest.cpp` | Protocol tests, mostly `static_assert`s | no |
| `apps/e727_spi_check.cpp` | Hardware bring-up diagnostic | yes |
| `cmake/FetchLibFt4222.cmake` | Unpacks the SHA256-pinned SDK zip when `PI_FETCH_FT4222` is on | not applicable |

Build instructions and SDK setup are in [`../README.md`](../README.md).

## Usage

```cpp
#include "E727SpiLink.hpp"

e727::SpiController controller;  // device 0, travel 0 to 35 mrad
controller.closeLoop();

const auto reply = controller.moveTo(e727::Milliradians{.x = 17.5, .y = 18.0});
if (!reply) {
    std::cerr << e727::describe(reply.error()) << '\n';
} else if (reply->checksumValid) {
    use(reply->position);
}
```

- Link against `e727::spi` from CMake, or build directly with
  `g++ -std=c++23 -O2 -Icpp/e727/include app.cpp -lft4222`.
- On a cold controller, settle at the centre in open loop before closing the loops, as
  `MotionSession` in the diagnostic does: send each step twice and wait 330 ms.
- Errors come back as `std::expected`, never exceptions. Only the constructor throws:
  `std::invalid_argument` for a bad range, `std::runtime_error` if the device cannot be opened.

## Units and travel range

- The wire carries positions in the unit the controller is configured for. The HISPEC E-727 is
  configured for 0 to 35 mrad, so `e727::Milliradians` is the native type. `e727::Microradians`
  is accepted by `moveTo` and converted on the way in.
- `e727::TravelRange` holds inclusive limits for both axes, defaulting to `kDefaultRange`
  (0 to 35 mrad). Pass a different range to the `SpiController` constructor for another
  configuration or a narrower safe window.
- `moveTo` refuses targets outside the range, and NaN, with `Error::OutOfRange` and sends
  nothing. It never clamps. `exchange()` sends raw frames unchecked, for diagnostics only.
- A new controller starts with an open-loop frame at the centre of its range, so `position()` and
  `ping()` before any `moveTo` cannot command motion away from the centre.

## Protocol

Every transaction is one 14-byte frame each way, full duplex, with the same layout in both
directions. A reply reports the controller's state from before the frame it arrived with.

| Bytes | Field | Value |
|---|---|---|
| 0 | Protocol id | `0x10` |
| 1 | Axis count | `0x02` |
| 2 | Flags | `0x00` |
| 3 | Servo | `0x00` open loop, `0x03` closed loop |
| 4 to 7 | x position | IEEE-754 binary32, big-endian |
| 8 to 11 | y position | IEEE-754 binary32, big-endian |
| 12 to 13 | Checksum | CRC-16 over bytes 0 to 11, big-endian |

The CRC uses polynomial `0x8005`, initial value `0xFFFF`, no reflection and no final XOR. Frozen
vectors in `E727Protocol.hpp` run as `static_assert`s, so any change to the encoding fails the
build.

SPI settings: single IO, `CLK_DIV_4`, clock idle low, data on the trailing edge, slave select SS0.
The FT4222 system clock is read but never set, because setting it emits a spurious pulse on SCK,
CS and MOSI. `e727_spi_check` prints the clock rate every time it opens the device.

## e727_spi_check

Built as `cpp/build/e727/e727_spi_check`. Run `e727_spi_check --help` for every option.

Run the modes in order, because each assumes the one before it passed. The frame layout and
CRC vectors are checked at compile time, so if the tool built, those passed.

| Mode | Needs | Mirror | Checks |
|---|---|---|---|
| `probe` | FT4222 | none | Device is visible and opens as SPI master; prints the clock |
| `readback` | controller | recentres in open loop | Every transfer completes and every reply checksum is valid |
| `move` | controller | steps one axis across the range | Readback follows command with slope 1; shows which physical axis moved |
| `speed` | controller | holds the centre | No errors, rate above the 500 Hz tracking loop, p99 transfer within its 2 ms budget |

```bash
e727_spi_check probe
e727_spi_check readback
e727_spi_check move --axis y
e727_spi_check speed --duration 600
```

- Modes that move the mirror ask for confirmation unless `--yes` is given, and every target is
  checked against `--range`.
- Ctrl-C stops a run early and parks the mirror at the centre with the loops open.

## Changing settings

Every setting is a named `constexpr` or a constructor or command-line argument. Use the run-time
options where they exist, and edit constants only when the controller's configuration changes.
After any edit, rebuild, run `ctest`, then run the `e727_spi_check` modes in order.

### At run time, no rebuild

| Setting | In code | In `e727_spi_check` |
|---|---|---|
| Travel range | `SpiController{deviceIndex, TravelRange{.min = 2.0, .max = 33.0}}` | `--range 2:33` |
| Which FT4222 | `deviceIndex`, the first constructor argument | `--device 1` |
| Custom USB ids | `registerUsbId(vendorId, productId)` before constructing | `--pid` |
| Axis to sweep, length of a speed run | not applicable | `--axis`, `--duration` |

### Default travel range

`kDefaultRange` in `include/E727Protocol.hpp`. Change it only if the controller is reconfigured,
since a narrower window for one application belongs in the constructor argument instead.
`tests/E727ProtocolTest.cpp` asserts the 0 to 35 mrad centre and span, so update it alongside.

### Protocol, in `include/E727Protocol.hpp`

| What | Where |
|---|---|
| Frame size and byte offsets | `kFrameSize`, `kProtocolIdByte` to `kChecksumByte` |
| Fixed header bytes | `kProtocolId`, `kAxisCount`, `kNoFlags` |
| Servo codes | `enum class Servo` |
| CRC | `kCrcInit`, `kCrcPolynomial` |
| Wire unit | `buildFrame` and `positionOf` take and return `Milliradians` |

The frozen vectors at the bottom of the header are `static_assert`s, so any encoding change fails
the build on purpose. GCC prints what the new code computes, in decimal, as the left side of
`note: the comparison reduces to '(30990 == 4660)'`. Update a vector only after
`e727_spi_check readback` shows the controller accepts the new encoding.

A controller configured in a unit other than milliradians needs the wire unit changed, not only
the range.

### SPI link, in `include/E727SpiLink.hpp`

`kSpiMode`, `kSpiClockDivider`, `kSpiClockPolarity`, `kSpiClockPhase` and `kSlaveSelectSs0`. The
SPI clock is the FT4222 system clock divided by `kSpiClockDivider`. The system clock itself is
never set, for the reason given under [Protocol](#protocol).

### Diagnostic tool, in `apps/e727_spi_check.cpp`

The constants at the top of the file:

| Constants | Affects |
|---|---|
| `kSettleRepeats`, `kSettleDelay` | Settling at the centre before closing the loops and when parking |
| `kReadbackFrames`, `kFramesToDump` | `readback` length and how many frames it prints in hex |
| `kSweepMarginFraction`, `kSweepPoints`, `kSamplesPerPoint`, `kSlopeTolerance` | `move` |
| `kTrackingLoopHz`, `kDefaultDurationSeconds` | `speed` pass threshold and default length |

## Not yet confirmed on hardware

- Reply checksum convention: `readback`
- Which physical axis each wire axis drives, and that the controller's scale matches 0 to 35 mrad:
  `move`
