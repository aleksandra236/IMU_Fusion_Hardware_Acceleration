# AHRS Fusion Implementation for HW/SW Co-Design

[![Based on](https://img.shields.io/badge/Based%20on-xioTechnologies%2FFusion-blue)](https://github.com/xioTechnologies/Fusion)
[![Language](https://img.shields.io/badge/Language-C99-green)]()
[![License](https://img.shields.io/badge/License-MIT-yellow)]()

## Overview

This project is a modified implementation of the [xioTechnologies Fusion](https://github.com/xioTechnologies/Fusion) AHRS (Attitude and Heading Reference System) algorithm, redesigned for **Hardware/Software co-design**. The algorithm has been partitioned to enable FPGA/VHDL acceleration of computationally intensive sections.

The Fusion library is a sensor fusion solution for Inertial Measurement Units (IMUs), based on the revised AHRS algorithm presented in [Madgwick's PhD thesis](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf).

## Architecture

The algorithm is divided into **11 sections** with a clear HW/SW boundary:

```
┌─────────────────────────────────────────────────────────────────┐
│                        SOFTWARE (CPU)                           │
├─────────────────────────────────────────────────────────────────┤
│  Section 1: Store Accelerometer                                 │
│  Section 2: Gyroscope Range Check                               │
│  Section 3: Ramp Gain (initialization)                          │
│  Section 4: Calculate Half Gravity Vector                       │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    HARDWARE ACCELERATOR (FPGA)                  │
├─────────────────────────────────────────────────────────────────┤
│  Section 5: Accelerometer Feedback (2x Fast InvSqrt)            │
│  Section 6: Gyroscope Conversion (deg/s → rad/s)                │
│  Section 7: Apply Feedback                                      │
│  Section 8: Quaternion Integration                              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                        SOFTWARE (CPU)                           │
├─────────────────────────────────────────────────────────────────┤
│  Section 9:  Quaternion Normalization (Fast InvSqrt)            │
│  Section 10: Zero Heading (during initialization)               │
│  Section 11: Output Euler Angles                                │
└─────────────────────────────────────────────────────────────────┘
```

### Sample-by-Sample Processing

The hardware accelerator processes one sample at a time (triggered by a START signal from the CPU). This approach:
- Matches the VP TLM-2.0 implementation where each `start_event` corresponds to one sample
- Eliminates batch buffering — CPU sends one sample, waits for `ahrs_done`, then normalizes
- Directly maps to the VHDL FSM design where each sample triggers one hardware execution cycle

## Profiling Results

### Wall-clock timing (`ahrs_profile_rt`)

Measures accumulated wall-clock time per section group over 1986 real samples:

| Section group | Time (µs) | Share |
|---|---|---|
| SW Sections 1–4 | 55.1 | 18.9% |
| HW Sections 5–8 | 142.9 | 49.1% |
| SW Sections 9–11 | 92.9 | 31.9% |
| **Total** | **290.9** | — |
| Per sample | 0.147 µs | — |

### Instruction count (`ahrs_embedded` + Callgrind)

Callgrind instruction-level profile over 1986 samples (720,981 total instructions):

| Function | Instructions | Share |
|---|---|---|
| `AhrsHW_Sections5to8` | 254,376 | 35.3% |
| `main` (SW sections 1–4, 9–11) | 210,510 | 29.2% |
| `atan2f` / `atanf` (libc math) | ~37,000 | ~5.1% |
| dynamic linker / startup | ~130,000 | ~18% |

The HW accelerator (Sections 5–8) dominates algorithmic cost — validating the HW/SW partitioning choice.

## spec/ — Source Files

| File | Purpose |
|---|---|
| `src/main.c` | Production binary — reads `.bin` sensor file, writes orientation CSV |
| `src/main_embedded.c` | Profiling (Callgrind) — all 1986 sensor samples compiled in as a C array (`sensor_data_embedded.h`), zero file I/O at runtime → only algorithmic instructions counted |
| `src/main_profile_realtime.c` | Profiling (wall-clock) — same algorithm, wraps each section group with `clock_gettime()` to measure real execution time per section; prints breakdown at end |
| `src/ahrs_hw.c` | HW accelerator (Sections 5–8) — called by all three binaries above; this is the function that would map to FPGA/VHDL |
| `src/csv_to_bin.c` | Pre-processing tool — converts CSV sensor data to the binary format that `ahrs_pipeline` reads |
| `src/visualize_motion.py` | 3D visualization — takes the orientation CSV output (time, qw, qx, qy, qz, roll, pitch, yaw) and renders an animated 3D box showing the sensor orientation over time, plus roll/pitch/yaw plots |
| `include/ahrs_hw.h` | Shared types and interface for the HW accelerator |
| `include/sensor_data_embedded.h` | 1986 sensor samples embedded as a C array (used by `main_embedded.c` and `main_profile_realtime.c`) |

## Project Structure

```
y26-g05/
├── README.md
├── .gitignore
├── spec/                          # C99 reference implementation
│   ├── Makefile                   # Build system
│   ├── include/
│   │   ├── ahrs_hw.h              # HW accelerator interface (Sections 5-8)
│   │   └── sensor_data_embedded.h # Compile-time embedded sensor data
│   └── src/
│       ├── ahrs_hw.c              # HW accelerator (Sections 5-8)
│       ├── main.c                 # Production: reads .bin sensor file, writes orientation CSV
│       ├── main_embedded.c        # Profiling: sensor data compiled-in (sensor_data_embedded.h), zero file I/O at runtime → clean Callgrind target
│       ├── main_profile_realtime.c # Profiling: simulates real-time sample arrival (sleep/timer between samples) to measure per-function timing under realistic scheduling
│       ├── csv_to_bin.c           # Pre-processing tool: converts CSV sensor data to binary
│       └── visualize_motion.py    # 3D orientation visualization
├── cpp_spec/                      # C++ floating-point reference implementation
├── analysis/                      # Analysis documents and fixed-point analysis
├── data/                          # Sensor input data & reference artifacts
│   ├── sensor_data_full.csv       # Real recorded IMU data (1986 samples, 100 Hz) — primary input
│   ├── hls_analysis/              # Vivado HLS synthesis reports
│   │   └── ahrs_ip_top_csynth.rpt
│   └── profiling_results/         # Callgrind profiling outputs & screenshots
│       ├── callgrind.out.7769
│       ├── callgrind_section5to8.out
│       ├── Rezultati_profajliranja.png
│       └── profajliranje_v2.png
└── vp/                            # SystemC/TLM-2.0 Virtual Platform
    ├── Makefile
    ├── README.md
    ├── sc_main.cpp                # Top-level instantiation
    ├── cpu.cpp / cpu.hpp          # CPU model (SW Sections 1-4, 9-11)
    ├── ahrs_ip.cpp / ahrs_ip.hpp  # AHRS IP model (HW Sections 5-8)
    ├── bram.cpp / bram.hpp        # BRAM model
    ├── ddr.cpp / ddr.hpp          # DDR model
    ├── spi.cpp / spi.hpp          # SPI controller model
    ├── sensor.cpp / sensor.hpp    # Sensor model
    ├── interconnect.cpp / interconnect.hpp # TLM router
    ├── defines.hpp                # Shared constants and types
    ├── ahrs_ip_hls.cpp            # Vivado HLS reference implementation
    └── vp_output.csv              # VP simulation output
```

## Building

### Prerequisites

- GCC compiler (C99 support)
- Make
- Valgrind (for profiling)
- Python 3 with matplotlib, numpy, pandas, scipy (for visualization)

### Build Commands

```bash
cd spec

# Build all working targets
make ahrs_pipeline ahrs_embedded ahrs_profile_rt

# Build production binary (reads .bin sensor file at runtime)
make ahrs_pipeline

# Build embedded version (all sensor data compiled-in; best for Callgrind)
make ahrs_embedded

# Build real-time profiling version (simulates sensor arrival timing)
make ahrs_profile_rt

# Clean build artifacts
make clean
```

## Usage

### Basic Workflow

```bash
cd spec

# Run AHRS processing (embedded sensor data, no file I/O)
./ahrs_embedded

# Run with binary input file
./ahrs_pipeline ../data/sensor_data_full.csv.bin orientation_data.csv

# Visualize results
python3 src/visualize_motion.py orientation_data.csv
```

> To convert a CSV to binary format: `./csv_to_bin <input.csv> <output.bin>` (build with `make csv_to_bin`)

### Quick Run (production + visualization)

```bash
cd spec
make ahrs_embedded && ./ahrs_embedded
```

### Profiling

```bash
cd spec

# Instruction-count profiling with Callgrind (use ahrs_embedded — no file I/O noise)
make ahrs_embedded
valgrind --tool=callgrind --callgrind-out-file=callgrind.out ./ahrs_embedded
kcachegrind callgrind.out

# Wall-clock timing per section (SW 1-4, HW 5-8, SW 9-11)
make ahrs_profile_rt
./ahrs_profile_rt

# View existing profiling results
kcachegrind ../data/profiling_results/callgrind.out.7769
```

## Data Formats

### Input: Sensor Data CSV

```csv
time,gx,gy,gz,ax,ay,az
0.000000,0.046148,-0.109713,0.123044,-0.024396,0.003533,1.006581
0.010000,0.228968,0.262713,-0.079755,-0.011761,0.002488,1.012061
...
```

- `time`: Timestamp in seconds
- `gx, gy, gz`: Gyroscope readings (deg/s)
- `ax, ay, az`: Accelerometer readings (g)

### Output: Orientation CSV

```csv
time,qw,qx,qy,qz,roll,pitch,yaw
0.000000,1.000000,0.000000,0.000000,0.000000,0.000,0.000,0.000
0.010000,0.999998,0.000143,0.000603,-0.000000,0.016,0.069,0.000
...
```

- `qw, qx, qy, qz`: Orientation quaternion
- `roll, pitch, yaw`: Euler angles (degrees)

## Configuration

Key parameters in `spec/include/ahrs_hw.h` and source files:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SAMPLE_RATE` | 100 Hz | IMU sampling rate |
| `gain` | 0.5 | Algorithm gain (0 = gyro only) |
| `gyroscopeRange` | 2000 °/s | Gyroscope measurement range |
| `accelerationRejection` | 10° | Threshold for accel rejection |
| `recoveryTriggerPeriod` | 500 samples | Recovery trigger period |
## Virtual Platform (VP)

The SystemC/TLM-2.0 virtual platform in `vp/` is fully implemented and simulates the complete HW/SW co-design:

```bash
cd vp

# Build
make

# Run simulation (produces vp_output.csv)
make run

# Compare VP output against float reference
make compare && ./compare
```

See [vp/README.md](vp/README.md) for the full block diagram, module descriptions, and simulation details.
## Algorithm Features

Based on the original Fusion library:

- **AHRS Algorithm**: Combines gyroscope and accelerometer data for 3D orientation
- **Fast Inverse Square Root**: Optimized normalization using Pizer's implementation
- **Gyroscope Bias Estimation**: Runtime offset compensation
- **Acceleration Rejection**: Filters out linear/rotational accelerations
- **Angular Rate Recovery**: Handles gyroscope saturation
- **Initialization Ramping**: Smooth convergence from arbitrary initial state

## Visualization

The `visualize_motion.py` script provides:

1. **Euler Angles Plot**: Roll, pitch, yaw over time
2. **3D Orientation Trajectory**: Path through orientation space
3. **3D Animation**: Real-time box rotation visualization

```bash
python3 src/visualize_motion.py orientation_data.csv
```

## Future Work (vp/)

The `vp/` directory is reserved for VHDL/RTL implementation:

- [ ] Implement FSM for batch processing
- [ ] Design fixed-point arithmetic units
- [ ] Implement Fast Inverse Square Root in hardware
- [ ] Create AXI/Avalon bus interface for SW communication
- [ ] Synthesis and timing analysis

## References

- [Original Fusion Library](https://github.com/xioTechnologies/Fusion)
- [Madgwick's PhD Thesis](https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf)
- [Fast Inverse Square Root](https://en.wikipedia.org/wiki/Fast_inverse_square_root)
- [Pizer's Implementation](https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/)

## License

This project is based on the [xioTechnologies Fusion](https://github.com/xioTechnologies/Fusion) library, which is licensed under the MIT License.

## Authors

College project - Group y26-g05
