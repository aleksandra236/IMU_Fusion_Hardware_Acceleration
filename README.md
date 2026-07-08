# IMU_Fusion_Hardware_Acceleration

# Hardware Accelerator for Madgwick IMU Filter on Zybo Z7-10

**Target Device:** Xilinx Zynq-7000 (xc7z010clg400-1)  
**Development Board:** Zybo Z7-10  
**Design Methodologies:** ESL (Vitis HLS/SystemC) vs. Custom RTL (VHDL)  

---

## 1. Project Overview
This project presents the design, implementation, and performance evaluation of a hardware accelerator dedicated to the Madgwick IMU sensor fusion filter (specifically focusing on Sections 5 through 8). The main objective is to optimize resource utilization and computational latency on a resource-constrained FPGA platform by comparing Electronic System Level (ESL) high-level synthesis with a fully custom Register-Transfer Level (RTL) architecture.

---

## 2. Hardware Architecture & Design Methodologies

### 2.1 ESL Approach (Vitis HLS / SystemC)
* **Implementation:** The core Madgwick filter algorithm was modeled using SystemC.
* **Data Type Optimization:** Floating-point variables were replaced with the `sc_fixed` data type to optimize hardware area and power consumption.
* **Pipelining:** High-Level Synthesis (HLS) optimization directives (pragmas) were applied to enforce pipelining and achieve minimal execution latency.

### 2.2 RTL Approach (VHDL)
* **Architecture:** Designed a fully custom hardware architecture using VHDL.
* **Hybrid Arithmetic Data Path:** Section 5 retains the IEEE 754 single-precision floating-point format (`float32`) utilizing three parallel floating-point multiplier (`fp_mul`) cores to preserve accuracy during initial orientation processing. Sections 6, 7, and 8 transition entirely to optimized fixed-point formats to minimize logic utilization.
* **4-Stage Pipeline & FSM:** Implemented a balanced 4-stage execution pipeline managed by an explicit Finite State Machine (FSM). The architecture successfully synchronizes the 4-clock cycle latency of the floating-point cores and resolves data hazards during dot product calculations.
* **DSP Optimization:** Constrained the `quat_t` fixed-point format to 18 bits. This constraint enables the 12 multipliers in Section 8 to map directly into individual **DSP48E1** slices without cascading, preventing exponential resource growth.
* **Synthesis Controls:** Incorporated explicit VHDL attributes (`keep` and `dont_touch`) along with explicit `shift_right` and `resize` operations. This prevents Vivado synthesis from incorrectly absorbing registers into DSP slices during global optimization passes.

---

## 3. System Integration & Verification
* **IP Packaging:** Both ESL and RTL accelerator variants are packaged as standard AXI4-Lite compliant IP cores.
* **Vivado IP Integrator:** The blocks are integrated with the Zynq Processing System (PS) via an AXI interconnect matrix.
* **Hardware-in-the-Loop (HIL) Testing:** Validated functionality and timing using standalone C applications executed on the ARM Cortex-A9 processor via Xilinx Vitis. Computational outputs were cross-checked directly against a golden software reference model.

---

## 4. Key Results

| Metric | ESL (Vitis HLS / SystemC) | Custom RTL (VHDL) |
| :--- | :--- | :--- |
| **Data Path** | Pure Fixed-Point (`sc_fixed`) | Hybrid (`float32` + 18-bit Fixed) |
| **Available DSPs (xc7z010)** | 2 Free DSP Slices | 25 Free DSP Slices |
| **DSP Optimization** | Cascaded Blocks | Single DSP48E1 Mapping |
| **Timing Closure** | Passed | Passed |

---
