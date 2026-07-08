#ifndef CPU_HPP
#define CPU_HPP

#include "defines.hpp"
#include <cmath>

/**
 * @brief CPU Module
 *
 * Orchestrates the AHRS processing pipeline sample-by-sample:
 *
 *   Loop:
 *     1. Wait for sensor_ready  (SPI has one raw sample ready)
 *     2. SW Phase 1 – SW_PrepareSample (Sections 1-4):
 *          - Store accelerometer
 *          - Gyroscope range check
 *          - Ramp gain (initialisation period)
 *          - Compute halfGravity from current quaternion state
 *          → Write prepared HwInputSample to BRAM_INPUT_OFFSET
 *     3. Write START bit to AHRS IP control register
 *     4. Wait for ahrs_done interrupt
 *     5. SW Phase 2 – SW_ProcessOutput (Sections 9-11):
 *          - Quaternion normalise (fast inverse sqrt)
 *          - Zero-heading correction (during initialisation)
 *          → Update internal quaternion state
 *     6. Store quaternion to DDR
 *     7. Repeat until MAX_SAMPLES, then stop.
 *
 * Socket layout:
 *   isock_interconnect → Interconnect::tsock_cpu
 *   isock_ddr          → DDR::tsock
 */
class CPU : public sc_core::sc_module { // defining CPU as a SystemC module 
public:
    SC_HAS_PROCESS(CPU);  // macro to enable SystemC processes in CPU module
    CPU(sc_core::sc_module_name name); // constructor declaration, definition in cpu.cpp

    tlm_utils::simple_initiator_socket<CPU> isock_interconnect; // initiator socket for Interconnect communication
    tlm_utils::simple_initiator_socket<CPU> isock_ddr; // initiator socket for DDR communication

private:
    void cpu_process(); // main process method that implements the CPU behavior, 
    //definition in cpu.cpp, will be registered as SC_THREAD in the constructor

    // ── SW Phases ────────────────────────────────────────────────────────────
    // Section 1-4: prepare one HwInputSample from raw sensor data
    void sw_prepare_sample(const RawSensorSample &raw,
                           float deltaTime,
                           HwInputSample &out);

    // Section 9-11: normalise unnormalized quaternion output
    void sw_process_output(const HwOutputSample &hwOut);

    // ── TLM helpers ──────────────────────────────────────────────────────────
    void  write_interconnect(int addr, float value); // send a write transaction to the Interconnect
    float read_interconnect(int addr); // send a read transaction to the Interconnect and return the float value read
    void  write_ddr(int addr, float value); // send a write transaction to the DDR

    // ── AHRS SW state (mirrors FusionAhrs from spec) ──────────────────────────
    // SW sekcije (1-4, 9-11) koriste float – identično sa C++ spec referencom
    float qw, qx, qy, qz;          // Current quaternion (normalised after Sec.9)
    float rampedGain;
    float rampedGainStep;
    float gain;
    float gyroscopeRange;
    bool  initialising; // flag indicating if we're still in the initialisation period (for ramped gain and zero-heading)
    bool  angularRateRecovery; // flag indicating if we're in angular rate recovery mode

    int sample_count;
};

#endif
