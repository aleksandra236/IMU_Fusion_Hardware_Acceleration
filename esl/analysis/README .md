# Fixed-Point Analysis (SystemC)

SystemC implementation of AHRS Sections 5-8 using `sc_fixed` types for **bitwidth-accurate** hardware modeling. This phase evaluates precision vs resource trade-offs before RTL implementation.

## Contents

```
analysis/
├── include/
│   └── ahrs_hw.h          # SystemC interface
├── src/
│   ├── ahrs_hw.cpp        # V3 Final - fixed-point (27 frac bits)
│   ├── main.cpp           # Test pipeline
│   └── visualize_motion.py # 3D visualization tool
├── results/               # Test outputs
│   ├── fixed_sensor_short_results.txt
│   └── fixed_sensor_full_results.txt
└── Makefile              # SystemC build system
```

---

## Prerequisites

```bash
# SystemC installation required
export SYSTEMC_HOME=/usr/local/systemc-2.3.3
export LD_LIBRARY_PATH=$SYSTEMC_HOME/lib-linux64:$LD_LIBRARY_PATH
```

---

## Building

```bash
make                # Build SystemC executable
make clean          # Remove build artifacts
```

---

## Running Tests

```bash
# Short test (499 samples)
./ahrs_pipeline ../data/sensor_data_short.csv orientation_short.csv

# Full test (1986 samples)
./ahrs_pipeline ../data/sensor_data_full.csv orientation_full.csv
```

---

## Type Configuration

See `docs/type_definitions.md` for complete type specifications.

**V3 Final (Current):**
- Quaternion: 29-bit (27 frac) → `sc_fixed<29, 2, SC_RND, SC_SAT>`
- dq: 27-bit (23 frac) → `sc_fixed<27, 5, SC_RND, SC_SAT>`
- Max error: **1.37°** @ 2000 samples

To modify types, edit `src/ahrs_hw.cpp` (lines 20-58).

---

## Results

| Test | Samples | Max Error | Status |
|------|---------|-----------|--------|
| Short | 499 | 0.02° | ✓ Excellent |
| Full | 1986 | 1.37° | ✓ Acceptable |

Detailed analysis: `docs/fixed_point_analysis.md`

---

## Visualization

```bash
python3 src/visualize_motion.py orientation_output.csv
```

Generates:
- Euler angle plots (roll, pitch, yaw vs time)
- 3D orientation trajectory
- Animated box rotation (optional)

---

## Purpose

This analysis phase determines **optimal bitwidths** for VHDL implementation (`rtl/`), ensuring:
1. Acceptable accuracy (< 2° error)
2. Minimal FPGA resources
3. Real-time performance (< 100 cycles/sample)
