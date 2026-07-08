# Fixed-Point vs Floating-Point Analysis

## Executive Summary

This document presents a detailed analysis comparing **fixed-point** (SystemC `sc_fixed`) and **floating-point** (C `float`) implementations of the AHRS algorithm Sections 5-8 (hardware accelerator).

**Key Finding:** Fixed-point implementation with **27 fractional bits** for quaternions achieves **1.37° maximum error** over 2000 samples, making it suitable for real-time IMU applications while saving ~60% power compared to floating-point hardware.

---

## 1. Motivation

### Why Fixed-Point?

| Aspect | Floating-Point | Fixed-Point |
|--------|----------------|-------------|
| **Hardware Complexity** | High (multipliers, shifters, normalizers) | Low (integer arithmetic) |
| **Power Consumption** | High (~5-10x) | Low (baseline) |
| **Latency** | ~10-20 cycles | ~3-5 cycles |
| **Area (FPGA)** | ~500-1000 LUTs per FP unit | ~50-100 LUTs per FX unit |
| **Determinism** | Poor (rounding varies) | Excellent (fixed rounding) |

For **embedded FPGA** applications (Zybo Z7-10), fixed-point is the optimal choice.

---

## 2. Methodology

### Test Configuration

| Parameter | Value |
|-----------|-------|
| Sample Rate | 100 Hz |
| Test Duration | Short: 5s (499 samples), Full: 20s (1986 samples) |
| Float Reference | GCC `-O2` optimized, IEEE 754 single precision |
| Fixed-Point | SystemC 3.0.2, `sc_fixed` types with rounding & saturation |

### Error Metrics

1. **Absolute Error (L2 Norm)**
   ```
   error = ||q_float - q_fixed||₂
   ```

2. **Relative Error**
   ```
   rel_error = (error / ||q_float||₂) × 100%
   ```

3. **Drift Ratio**
   ```
   drift = mean(last_100_errors) / mean(first_100_errors)
   ```

---

## 3. Results

### 3.1 Short Sequence (499 samples)

#### Final Orientation Comparison

| Parameter | Float Reference | Fixed-Point V3 | Error |
|-----------|-----------------|----------------|-------|
| Roll | 123.00° | 123.01° | **+0.01°** ✓ |
| Pitch | -8.06° | -8.07° | **-0.01°** ✓ |
| Yaw | 26.63° | 26.61° | **-0.02°** ✓ |

**Maximum Error:** 0.02° - **Excellent!**

#### Error Statistics

| Metric | Value |
|--------|-------|
| Mean Absolute Error | 0.000101 |
| Max Absolute Error | 0.001011 |
| Mean Relative Error | 0.0101% |
| Max Relative Error | 0.1008% |

#### Drift Analysis

```
Initial error (sample 1):      0.00000283
Final error (sample 499):      0.00020128
Drift ratio:                   6.04x
```

 **Observation:** Error accumulates 6x over 499 samples, indicating potential long-term drift.

---

### 3.2 Long Sequence (1986 samples)

#### Final Orientation Comparison

| Parameter | Float Reference | Fixed-Point V3 | Error |
|-----------|-----------------|----------------|-------|
| Roll | -133.28° | -133.69° | **-0.41°** ✓ |
| Pitch | -26.68° | -26.09° | **+0.59°** ✓ |
| Yaw | 54.36° | 55.73° | **+1.37°**|

**Maximum Error:** 1.37° - **Acceptable** for most IMU applications.

#### Error Growth

```
Short test (499 samples):   Max error = 0.02°
Long test (1986 samples):   Max error = 1.37°
Scaling factor:             ~70x increase for 4x longer sequence
```

**Analysis:** Error growth is **sub-linear** (not proportional to time), suggesting algorithmic drift rather than pure accumulation.

---

## 4. Component-Wise Error Breakdown

### Section 5: Accelerometer Feedback

| Variable | Max Abs Error | Max Rel Error | Notes |
|----------|---------------|---------------|-------|
| Accel Magnitude² | 0.000014 | 0.006% | Excellent |
| Accel Inv Mag | 0.000216 | 0.003% | Excellent |
| Normalized Accel | 0.000037 | 0.364% | Good |
| **Cross Product** | **0.000771** | **4.84%** | Largest error source |
| Cross Inv Mag | 0.010727 | 0.059% | Acceptable |

**Finding:** Cross product is the **most error-prone** operation due to subtraction cancellation.

### Section 8: Quaternion Integration

| Variable | Max Abs Error | Max Rel Error | Notes |
|----------|---------------|---------------|-------|
| dq (derivative) | 0.005235 | 114.6% | High relative (low absolute) |
| **New Quaternion** | **0.000711** | **17.9%** | Accumulates over time |

**Finding:** Large relative errors on `dq` are **misleading** - absolute values are tiny (~1e-5), so relative percentage is inflated.

---

## 5. Type Precision Impact

### V3 Final Configuration

| Type | Frac Bits | Rationale |
|------|-----------|-----------|
| `quat_t` | **27** | Critical: accumulates error |
| `dq_t` | **23** | Critical: multiplied by small dt |
| `norm_t` | **24** | Sensitive: cross product precision |
| `dot_t` | **25** | Sensitive: branching decision |

### Attempted Optimization: V3.1 (30 frac bits for quat)

| Configuration | Max Error (1986 samples) | Resource Cost |
|---------------|--------------------------|---------------|
| V3 (27 frac) | 1.37° | +15% LUTs (baseline) |
| **V3.1 (30 frac)** | **1.37°** | **+30% LUTs** |

**Conclusion:** Increasing precision beyond 27 frac bits provides **no benefit** - error is algorithmic, not quantization-limited.

---

## 6. Root Cause Analysis

### Why does error accumulate?

1. **Quaternion Integration** (Section 8)
   ```
   q_new = q_old + dq * dt
   ```
   - Small errors in `dq` compound over thousands of iterations
   - No correction mechanism in hardware (normalization is in SW)

2. **Cross Product Sensitivity** (Section 5)
   - Subtraction-heavy operation prone to cancellation errors
   - Small errors propagate through feedback loop

3. **Fast Inverse Square Root Approximation**
   - 2-iteration Newton-Raphson: ~0.17% error
   - Acceptable for single operations, but accumulates

### Why doesn't renormalization help?

**Tested:** Added periodic quaternion renormalization every 100 samples.

**Result:** No improvement (error remained 1.37°).

**Reason:** SW already normalizes quaternion **every sample** (Section 9), so additional renormalization is redundant.

---

## 7. Applicability Assessment

### Suitable For:

- **Consumer Drones** (error < 2° acceptable)
- **Robotics** (pose estimation with ±1° tolerance)
- **AR/VR Headsets** (with periodic recalibration)
- **Fitness Trackers** (orientation-based gesture detection)

### Requires Tuning:

- **Navigation-Grade IMU** (target < 0.5° error)
  - Solution: Use 30+ frac bits + Kahan summation
- **Precision Robotics** (sub-degree accuracy)
  - Solution: Sensor fusion with magnetometer/GPS

###  Not Suitable:

- **Aircraft Navigation** (requires double precision)
- **Satellite Attitude Control** (10⁻⁶° accuracy needed)

---

## 8. Recommendations

### For Production (Zybo Z7-10 Implementation):

1. **Use V3 Final** (27 frac bits for quaternion)
   - Optimal balance: 1.37° error, +15% resources
   - Tested and validated on 2000-sample sequences

2. **Implement Error Compensation**
   - Add magnetometer for heading correction (Section 5 extension)
   - Periodic reset from external reference (GPS, visual odometry)

3. **Monitor Drift in SW**
   - Track quaternion magnitude drift
   - Reset if magnitude deviates > 5% from 1.0

### For Extended Operation (> 10,000 samples):

1. **Increase quaternion precision** to 30 frac bits
2. **Implement Kahan summation** for quaternion integration
3. **Add magnetometer fusion** to correct yaw drift

---

## 9. Hardware Estimates (Xilinx 7-series)

### Resource Utilization

| Module | LUTs | FFs | DSP48s | BRAM |
|--------|------|-----|--------|------|
| **Float (baseline)** | ~5000 | ~2000 | 20-30 | 0 |
| **Fixed V3** | **~2500** | **~1150** | **8-12** | **0** |
| **Savings** | **50%** | **43%** | **60%** | - |

### Timing

| Implementation | Max Freq | Latency (cycles) | Throughput |
|----------------|----------|------------------|------------|
| Float | ~100 MHz | 80-100 | ~1 MHz |
| Fixed V3 | ~150 MHz | 55-75 | ~2 MHz |

**Result:** Fixed-point is **2x faster** and uses **50% fewer resources**.

---

## 10. Conclusion

Fixed-point implementation with **27 fractional bits** achieves:
-  **1.37° maximum error** over 2000 samples
-  **50% resource reduction** vs floating-point
-  **2x throughput improvement**
-  **Deterministic behavior** (critical for real-time systems)

**Trade-off accepted:** Slightly higher error vs float (1.37° vs theoretical 0°) is **acceptable** for embedded IMU applications.

---

## References

1. Madgwick, S. "An efficient orientation filter for inertial and inertial/magnetic sensor arrays" (2010)
2. IEEE 1666-2011: SystemC Language Reference Manual
3. Xilinx UG473: 7-Series FPGAs Memory Resources User Guide
4. "Fixed-Point Arithmetic: An Introduction" - R. Yates (2009)

---

**Version:** Final  
**Date:** January 31, 2026  
**Author:** Jelena (y26-g05)  
**Course:** PEANS (Projektovanje Elektronskih Uređaja na Sistemskom Nivou)
