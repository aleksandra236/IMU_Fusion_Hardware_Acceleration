# spec/ — C99 Reference Implementation

C99 implementation of the AHRS algorithm with HW/SW partitioning.  
Serves as the **golden reference** and profiling target for the project.

---

## Source Files

| File | Binary | Purpose |
|---|---|---|
| `src/main.c` | `ahrs_pipeline` | Production: reads `.bin` sensor file, writes orientation CSV |
| `src/main_embedded.c` | `ahrs_embedded` | Callgrind profiling: 1986 sensor samples compiled in as a C array — zero file I/O, only algorithmic instructions counted |
| `src/main_profile_realtime.c` | `ahrs_profile_rt` | Wall-clock profiling: wraps each section group (SW 1–4 / HW 5–8 / SW 9–11) with `clock_gettime()`, prints time breakdown per section |
| `src/ahrs_hw.c` | — | HW accelerator (Sections 5–8): called by all three binaries; maps to the FPGA/VHDL implementation |
| `src/csv_to_bin.c` | `csv_to_bin` | Pre-processing: converts CSV sensor data to binary format read by `ahrs_pipeline` |
| `src/visualize_motion.py` | — | 3D visualization: takes orientation CSV (time, qw, qx, qy, qz, roll, pitch, yaw) and renders an animated 3D box + roll/pitch/yaw plots |
| `include/ahrs_hw.h` | — | Shared types and HW accelerator interface |
| `include/sensor_data_embedded.h` | — | 1986 sensor samples as a C array (used by `ahrs_embedded` and `ahrs_profile_rt`) |

---

## Build

```bash
# Build all targets
make ahrs_pipeline ahrs_embedded ahrs_profile_rt csv_to_bin

# Clean
make clean
```

---

## Usage

### Production run

```bash
# Convert CSV to binary (one-time pre-processing)
./csv_to_bin ../data/sensor_data_full.csv sensor_data.bin

# Run AHRS and write orientation output
./ahrs_pipeline sensor_data.bin orientation_out.csv
```

### Visualize orientation output

```bash
python3 src/visualize_motion.py orientation_out.csv
```

Displays an animated 3D box showing sensor orientation over time and roll/pitch/yaw plots.  
`orientation_out.csv` is a generated file — run `ahrs_pipeline` first to produce it.

### Profiling

```bash
# Instruction-count profiling (Callgrind) — use ahrs_embedded, no file I/O noise
valgrind --tool=callgrind --callgrind-out-file=callgrind.out ./ahrs_embedded
kcachegrind callgrind.out

# Wall-clock timing per section group
./ahrs_profile_rt
```

---

## HW/SW Partitioning

```
SW (CPU)          HW (FPGA)          SW (CPU)
Sections 1–4  →  Sections 5–8   →  Sections 9–11
  pre-HW             core              post-HW
```

Each sample is processed one at a time. SW prepares inputs (Sections 1–4), calls `AhrsHW_Sections5to8()`, then normalizes and outputs the result (Sections 9–11).

---

## Profiling Results Summary

| Method | SW 1–4 | HW 5–8 | SW 9–11 |
|---|---|---|---|
| Wall-clock (`ahrs_profile_rt`) | 18.9% | **49.1%** | 31.9% |
| Instruction count (Callgrind) | 29.2% (`main`) | **35.3%** (`AhrsHW_Sections5to8`) | included in `main` |

HW sections dominate both metrics — confirms the partitioning decision.

For full profiling data and KCachegrind screenshots, see [`../data/profiling_results/`](../data/profiling_results/).
