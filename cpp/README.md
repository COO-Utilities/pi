# C++ interfaces

Native interfaces to PI controllers, independent of the Python package in the repository root.
Each device has its own directory with its own namespace, units, headers and apps.

| Device | Directory | Transport | Contents |
|---|---|---|---|
| PI E-727 (fast steering mirror) | [`e727/`](e727/README.md) | FTDI FT4222H USB-to-SPI | header-only interface, `e727_spi_check` diagnostic |

## Requirements

- Linux, GCC 13 or newer (C++23), CMake 3.25 or newer
- An FTDI FT4222H USB-to-SPI bridge between the host and the controller, and FTDI's LibFT4222 SDK
  to drive it (see below), tested with 1.4.4.232. Other USB-to-SPI adapters are not supported

## LibFT4222 SDK

The SPI link (`E727SpiLink.hpp`) and every tool link against FTDI's LibFT4222, a closed-source
library that bundles FTDI's D2XX driver. It is not included in this repository and has to be
installed on each machine that talks to hardware.

Without it, CMake prints `LibFT4222 not found under FT4222_ROOT=...` and builds the protocol
header and its tests only. That is enough for development away from the hardware.

### Download

Get "LibFT4222 Linux" from the [FT4222H product page](https://ftdichip.com/products/ft4222h/) in a
web browser. The FTDI site blocks scripted downloads, so `curl` and `wget` receive an HTML page
instead of the archive. Version 1.4.4.232 arrives as `libft4222-linux-1.4.4.232.zip`, which
contains a `.tgz`:

```bash
unzip libft4222-linux-1.4.4.232.zip
mkdir libft4222-1.4.4.232
tar -xzf libft4222-linux-1.4.4.232.tgz -C libft4222-1.4.4.232
```

### Install system-wide (recommended)

```bash
cd libft4222-1.4.4.232
sudo ./install4222.sh
sudo ldconfig
```

`install4222.sh` copies the library to `/usr/local/lib` and `libft4222.h`, `ftd2xx.h` and
`WinTypes.h` to `/usr/local/include`, which is where CMake looks by default. It does not run
`ldconfig`, so run that yourself or programs outside the build tree will not find the library.

### Use an unpacked copy without root

The archive keeps each architecture's library in `build-<arch>/` under a versioned name. Link the
one for your machine to the top of the directory, then point `FT4222_ROOT` at it:

```bash
cd libft4222-1.4.4.232
ln -s build-x86_64/libft4222.so.1.4.4.232 libft4222.so
cmake -S cpp -B cpp/build -DFT4222_ROOT=/path/to/libft4222-1.4.4.232
```

CMake records the directory in the build tree's RPATH, so the built apps run without
`LD_LIBRARY_PATH`.

### Use the pinned archive, no install

With `PI_FETCH_FT4222=ON`, CMake unpacks the downloaded zip into the build tree and uses it in
place of any installed copy. The archive's SHA256 is pinned in
[`e727/cmake/FetchLibFt4222.cmake`](e727/cmake/FetchLibFt4222.cmake), so configuration stops
with a hash mismatch on any other file, and every host builds against the same library.

```bash
cmake -S cpp -B cpp/build -DPI_FETCH_FT4222=ON \
      -DFT4222_SDK_URL=/absolute/path/to/libft4222-linux-1.4.4.232.zip
```

`FT4222_SDK_URL` also accepts an `https://` URL, but not FTDI's own, which serves a browser
challenge page to scripts. To move to a newer SDK, update `FT4222_SDK_VERSION` and
`FT4222_SDK_SHA256` together in that file.

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
| `PI_FETCH_FT4222` | `OFF` | Unpack the SHA256-pinned SDK zip at `FT4222_SDK_URL` instead of searching `FT4222_ROOT` |
| `FT4222_SDK_URL` | empty | Absolute path or URL of the SDK zip, required when `PI_FETCH_FT4222` is `ON` |
| `PI_WARNINGS_AS_ERRORS` | `OFF` | Build with `-Werror` |
| `CMAKE_BUILD_TYPE` | `Release` | Timing diagnostics are only meaningful from an optimised build |

## Install

```bash
sudo cmake --install cpp/build
```

This installs the headers to `include/e727`, `e727_spi_check` to `bin` and the `pi_cpp` CMake
package under the install prefix (`/usr/local` by default). Without LibFT4222, or with
`PI_FETCH_FT4222=ON`, only `E727Protocol.hpp` and `e727::protocol` are installed. Consumers link
the targets:

```cmake
find_package(pi_cpp 0.1 REQUIRED)
target_link_libraries(my_app PRIVATE e727::spi)  # or e727::protocol without the SDK
```

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
