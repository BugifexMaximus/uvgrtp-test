# uvgrtp-test

testing uvgrtp functionalities

## Building the sender example

The project is configured for C++20 and depends on OpenCV, FFmpeg and uvgRTP
development packages. On Debian-based systems you can install the prerequisites
with:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config \
    libopencv-dev libavcodec-dev libavutil-dev libswscale-dev libavformat-dev \
    libuvgrtp-dev
```

Then configure and build the sender sample:

```bash
cmake -S . -B build
cmake --build build
```

The sender executable will be available as `build/sender`.

### Building against a local uvgRTP checkout

If you have the `uvgRTP` sources cloned next to this project (for example via

```bash
git clone https://github.com/ultravideo/uvgRTP.git
```

the CMake build will automatically compile and link against that tree instead
of the system-wide `libuvgrtp` installation. This is convenient when testing
local modifications to uvgRTP: just re-run the build and the sender will link
against the freshly built library without additional configuration.
