# C++ interfaces

Native interfaces to PI controllers, independent of the Python package in the repository root.
Each device has its own directory with its own namespace, units, headers and tools.

| Device | Directory | Transport | Contents |
|---|---|---|---|
| PI E-727 (fast steering mirror) | [`e727/`](e727/README.md) | FTDI FT4222H USB-to-SPI | header-only interface, `e727_spi_check` diagnostic |

## Requirements

- Linux, GCC 13 or newer (C++23), CMake 3.25 or newer
- FTDI LibFT4222 SDK for anything that touches hardware, tested with 1.4.4.232. The default
  search location is `/usr/local` (`include/libft4222.h`, `include/ftd2xx.h`,
  `include/WinTypes.h`, `lib/libft4222.so`), which is where the SDK's `install4222.sh` puts it.
  Point `FT4222_ROOT` at an unpacked SDK directory to use a copy elsewhere.

Without the SDK, CMake builds the protocol headers and their tests only.

## Build

From the repository root:

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build
ctest --test-dir cpp/build
```

| CMake option | Default | Purpose |
|---|---|---|
| `FT4222_ROOT` | `/usr/local` | Install prefix or unpacked directory of the LibFT4222 SDK |
| `PI_WARNINGS_AS_ERRORS` | `OFF` | Build with `-Werror` |
| `CMAKE_BUILD_TYPE` | `Release` | Timing diagnostics are only meaningful from an optimised build |

## USB access without sudo

D2XX opens the device through usbfs, so a non-root user needs a udev rule. `601c` is the FT4222H
default product id; a board with a custom EEPROM needs the id that `lsusb` reports instead.

```
# /etc/udev/rules.d/99-ft4222.rules
SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="601c", GROUP="plugdev", MODE="0660"
```

Then reload with `sudo udevadm control --reload-rules && sudo udevadm trigger`, and add the user to
`plugdev`.

## Adding a device

Create `cpp/<device>/` with its own `CMakeLists.txt`, `include/` and namespace, add it to
`cpp/CMakeLists.txt` with `add_subdirectory`, and add a row to the table above. Units and limits
belong to the device directory; nothing is shared between devices.
