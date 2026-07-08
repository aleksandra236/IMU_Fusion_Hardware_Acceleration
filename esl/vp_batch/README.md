# AHRS Virtual Platform (VP)

SystemC/TLM-2.0 virtual platform for the **Attitude and Heading Reference System (AHRS)** hardware accelerator. The VP faithfully models the SW/HW co-design split of the Fusion AHRS algorithm and produces bit-accurate results matching the C++ specification.

---

## Block Diagram

```
  ┌──────────┐   SPI     ┌─────────────┐   tsock_cpu   ┌──────────────────┐
  │  Sensor  │ ◄──────── │     SPI     │               │       CPU        │
  └──────────┘           │  Controller │               │  (SW Sec. 1–4    │
                         └──────▲──────┘               │   & Sec. 9–11)   │
                       isock_spi │                      └──────┬───────────┘
                                │                              │ isock_ddr
                         ┌──────┴───────┐                ┌────▼────┐
                         │ Interconnect │                │   DDR   │
                         │ (TLM router) │                └─────────┘
                         └──┬───────┬───┘
                   isock_bram│       │isock_ip
                             ▼       ▼
                    ┌──────────┐  ┌───────────────────────┐
                    │  BRAM    │  │       AHRS IP          │
                    │ Port A   │  │  (HW Sections 5–8)    │
                    │          │  │                        │
                    │ Port B ◄─┼──┤ isock_bram (direct)    │
                    └──────────┘  └───────────────────────┘
```

Interrupt path: `AHRS_IP` → `ahrs_done` (sc_event) → `CPU`  
Sensor-ready path: `SPI` → `sensor_ready` (sc_event) → `CPU`

---

## Algorithm: SW/HW Split

The Fusion AHRS algorithm is partitioned across software (CPU) and hardware (AHRS IP):

| Phase | Where | Sections | Description |
|-------|-------|----------|-------------|
| Prepare | CPU (SW) | 1–4 | Store accel/gyro, range check, ramp gain, compute `halfGravity` from current quaternion |
| Accelerate | AHRS IP (HW) | 5–8 | Accel feedback (fast-inv-sqrt, cross product, dot-product check), gyro conversion, feedback application, quaternion integration → **unnormalised output** |
| Normalise | CPU (SW) | 9–11 | Fast-inv-sqrt normalisation (magic `0x5F1F1412`), optional zero-heading correction, quaternion state update |

---

## Module Overview

| File | Module | Role |
|------|--------|------|
| `sensor.hpp/cpp` | `Sensor` | IMU sensor model; responds to SPI READ requests with gyro/accel data |
| `spi.hpp/cpp` | `SPI` | SPI controller; collects 5 samples per batch from Sensor, buffers them internally, fires `sensor_ready`. CPU reads buffered data through Interconnect |
| `cpu.hpp/cpp` | `CPU` | Runs SW Sections 1–4 and 9–11; orchestrates the full pipeline |
| `ddr.hpp/cpp` | `DDR` | CPU-private DDR (1024 floats); direct connection, not routed through Interconnect |
| `interconnect.hpp/cpp` | `Interconnect` | TLM-2.0 address router: `0x0000–0x01FF` → BRAM, `0x0200–0x020F` → AHRS IP, `0x0300–0x031F` → SPI |
| `bram.hpp/cpp` | `Bram` | Dual-port BRAM (512 × 32-bit float words); Port A via Interconnect, Port B direct to AHRS IP |
| `ahrs_ip.hpp/cpp` | `AHRS_IP` | HW accelerator; executes Sections 5–8, writes unnormalised quaternions to BRAM, fires `ahrs_done` |
| `defines.hpp` | – | Memory map, data structures (`RawSensorSample`, `HwInputSample`, `HwOutputSample`), global event declarations |
| `sc_main.cpp` | – | Top-level: instantiates all 7 modules and binds all TLM sockets |
| `compare.cpp` | – | Standalone validation tool: runs the VP algorithm in pure C++ against `sensor_data_short.csv` and compares to reference results |

---

## Memory Map (BRAM word addresses)

| Region | Offset | Size | Description |
|--------|--------|------|-------------|
| Control registers | `0x0000` | 16 words | IP control/status |
| AHRS state | `0x0010` | 16 words | Quaternion state |
| Raw sensor buffer | `0x0020` | 60 words | CPU → BRAM (read from SPI): 10 × `RawSensorSample` (6 floats each) |
| HW input buffer | `0x005C` | 110 words | CPU → BRAM: 10 × `HwInputSample` (11 floats each) |
| HW output buffer | `0x00CA` | 40 words | IP → BRAM: 10 × `HwOutputSample` (4 floats each, unnormalised) |
| AHRS IP registers | `0x0200` | 16 words | Mapped through Interconnect |
| SPI controller | `0x0300` | 32 words | Mapped through Interconnect |

---

## Data Structures

```cpp
struct RawSensorSample {           // Buffered in SPI, read by CPU via Interconnect
    float gx, gy, gz;             // Gyroscope     (deg/s)
    float ax, ay, az;             // Accelerometer (g)
};  // 6 floats

struct HwInputSample {             // Written by CPU → BRAM_INPUT_OFFSET
    float gx, gy, gz;             // Gyroscope          (deg/s)
    float ax, ay, az;             // Accelerometer      (g)
    float halfGravityX, halfGravityY, halfGravityZ;  // From SW Sec. 4
    float deltaTime;              // Time step          (s)
    float rampedGain;             // Gain from SW Sec. 3
};  // 11 floats

struct HwOutputSample {            // Written by IP  → BRAM_OUTPUT_OFFSET
    float qw, qx, qy, qz;        // Unnormalised quaternion
};  // 4 floats
```

---

## Build & Run

**Requirements:** SystemC 2.3.4 (installed at `/usr/local/systemc`), GCC with C++17 support.

```bash
# Build the virtual platform
make

# Run the simulation (40 batches × 5 samples = 200 samples total)
make run

# Build standalone numerical comparison tool (run after make run)
make compare && ./compare

# Remove all build artifacts
make clean
```

### Simulation flow

The VP processes **200 samples** from `../analysis/data/sensor_data_short.csv` in **40 batches** of 5 samples each. Each batch follows the pipeline:

1. **SPI** reads 5 raw samples from Sensor → buffers them internally
2. **SPI** fires `sensor_ready` event → **CPU** wakes up
3. **CPU** reads raw sensor data from SPI (via Interconnect → SPI), processes each sample (SW Sec. 1–4) → writes `HwInputSample` to BRAM (`BRAM_INPUT_OFFSET`)
4. **CPU** writes to IP control register, starts AHRS IP
5. **AHRS IP** processes the batch with `sc_fixed` arithmetic (HW Sec. 5–8) → writes unnormalised quaternions to BRAM (`BRAM_OUTPUT_OFFSET`) → fires `ahrs_done`
6. **CPU** reads outputs and normalises (SW Sec. 9–11) → writes final quaternions to DDR

### Example simulation output (last batch)

```
[SPI] Batch 40 raw data buffered. Notifying CPU...
[CPU] *** Batch 40/40 ─ SW Phase 1 (Sections 1-4) ***
[CPU] Starting AHRS IP...
[AHRS_IP] Start signal received from CPU.
[AHRS_IP] Processing batch 40 (Sections 5-8)...
[AHRS_IP] Batch 40 done. Internal q=(...) [unnormalized]
[CPU] Interrupt received. SW Phase 2 (Sections 9-11)...
[CPU] Batch 40 done. Final q=(-0.00423, -0.0267, -0.999, 0.0318)
```

---

## Key Configuration (`defines.hpp`)

| Constant | Value | Description |
|----------|-------|-------------|
| `HW_BATCH_SIZE` | 5 | Samples processed per HW batch |
| `SAMPLE_RATE_HZ` | 100 | Sensor sample rate (Hz) |
| `MAX_BATCHES` | 40 | Number of batches (200 samples total) |
| `AHRS_IP_LATENCY_NS` | 13500 | HW latency: 1350 cycles × 10 ns clock (from HLS synthesis) |
| Initial `rampedGain` | 10.0 | Starting gain (ramps down to 0.5 over ~3 s) |

---

## Dependencies

- **SystemC 3.0.2** (or 2.3.4+) — `libsystemc`, headers and library at `/usr/local/systemc`
- **TLM-2.0** — bundled with SystemC
- **GCC ≥ 9** with `-std=c++17 -DSC_INCLUDE_FX`
- **libm** (`-lm`) — for `sinf`, `cosf`, `atan2f` in CPU SW sections

---

## AHRS IP Fixed-Point Types (`analysis/fixed_point_analysis.md` — V3 Final)

| Type | sc_fixed | Range | Frac bits | Used for |
|------|----------|-------|-----------|----------|
| `quat_t` | `<29,2>` | [-2, 2) | 27 | Quaternion components |
| `dot_t` | `<26,1>` | [-1, 1) | 25 | Dot product |
| `norm_t` | `<26,2>` | [-2, 2) | 24 | Normalised accel / cross product |
| `accel_t` | `<20,3>` | [-4, 4) | 17 | Accelerometer input |
| `hg_t` | `<26,2>` | [-2, 2) | 24 | Half-gravity vector |
| `halfgyro_t` | `<22,2>` | [-2, 2) | 20 | Half-angle gyro rates |
| `gain_t` | `<20,4>` | [0, 16) | 16 | rampedGain |
| `dt_t` | `<20,0>` | [0, 1) | 20 | deltaTime |

All sc_fixed types use `SC_RND` (round to nearest) and `SC_SAT` (saturate on overflow).

---

## HLS Latency

The `AHRS_IP_LATENCY_NS = 13500` value comes from Vitis HLS C synthesis targeting the Zybo Z7-10 board:
- **1350 clock cycles** for one batch of 5 samples (estimated by HLS)
- **10 ns clock period** (100 MHz)
- Full synthesis report: `data/hls_analysis/ahrs_ip_top_csynth.rpt`

The **2 ns delay** in `b_transport` for the AXI-Lite control register access is a model approximation. In a real system this would be determined from the AXI bus timing.

---

## Architecture Notes

- **SPI as target**: The SPI module acts as a TLM target (slave). The Interconnect routes CPU read requests to SPI, matching how a real Zynq SPI controller works as an AXI slave. SPI internally buffers raw sensor data and serves it to the CPU on demand.
- **Interconnect routes three targets**: BRAM (`0x0000–0x01FF`), AHRS IP (`0x0200–0x020F`), and SPI (`0x0300–0x031F`). Only the CPU initiates transactions through the Interconnect.
