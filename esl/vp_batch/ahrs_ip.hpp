#ifndef AHRS_IP_HPP
#define AHRS_IP_HPP

#include "defines.hpp"
#include <cmath>

// ============================================================================
// Fixed-Point Type Aliases (from fixed-point analysis – V3 Final)
// ============================================================================
typedef sc_fixed<29, 2, SC_RND, SC_SAT>  quat_t;      // quaternion [-2,2),  27 frac bits
typedef sc_fixed<26, 1, SC_RND, SC_SAT>  dot_t;       // dot product [-1,1), 25 frac bits
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  norm_t;      // norm. accel / cross, 24 frac bits
typedef sc_fixed<20, 3, SC_RND, SC_SAT>  accel_t;     // accel input [-4,4), 17 frac bits
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  hg_t;        // halfGravity [-2,2), 24 frac bits
typedef sc_ufixed<22, 2, SC_RND, SC_SAT> magsq_t;     // magnitude²  [0,4),  20 frac bits
typedef sc_ufixed<24, 6, SC_RND, SC_SAT> invmag_t;    // inv-mag     [0,64), 18 frac bits
typedef sc_fixed<22, 2, SC_RND, SC_SAT>  halfgyro_t;  // half-gyro  [-2,2), 20 frac bits
typedef sc_ufixed<20, 4, SC_RND, SC_SAT> gain_t;      // gain        [0,16), 16 frac bits
typedef sc_fixed<26, 5, SC_RND, SC_SAT>  adjhg_t;     // adj half-gyro, 21 frac bits
typedef sc_fixed<27, 5, SC_RND, SC_SAT>  dq_t;        // d-quaternion, 23 frac bits
typedef sc_ufixed<20, 0, SC_RND, SC_SAT> dt_t;        // deltaTime   [0,1),  20 frac bits

/**
 * @brief AHRS Hardware Accelerator (IP core) – Sections 5-8
 *
 * Implements EXACTLY the HW part described in the C++ spec (ahrs_hw.h /
 * AhrsHW_ProcessBatch):
 *
 *   For each HwInputSample in the batch:
 *     Section 5: Accelerometer feedback
 *                  – normalise accelerometer
 *                  – half-error = cross(accel, halfGravity)   [from SW Sec.4]
 *                  – scale error by rampedGain
 *     Section 6: Gyroscope conversion  deg/s → half-angle rates (rad/s × 0.5)
 *     Section 7: Apply feedback to gyroscope rates
 *     Section 8: Quaternion integration (first-order)
 *                  → write UNNORMALIZED quaternion to HwOutputSample
 *                  → update internal quaternion for next iteration
 *
 * The output quaternion is UNNORMALIZED – CPU normalises it in SW Sec.9.
 *
 * Socket layout:
 *   tsock_interconnect ← Interconnect::isock_ip   (control writes from CPU)
 *   isock_bram         → Bram::tsock_b             (direct data access)
 */
class AHRS_IP : public sc_core::sc_module {
public:
    SC_HAS_PROCESS(AHRS_IP);
    AHRS_IP(sc_core::sc_module_name name);

    tlm_utils::simple_target_socket<AHRS_IP>    tsock_interconnect;
    tlm_utils::simple_initiator_socket<AHRS_IP> isock_bram;

    void b_transport(tlm::tlm_generic_payload&, sc_core::sc_time&);

private:
    void solve();

    // Process one sample (Sections 5-8); updates q0..q3; returns unnormalized result
    void hw_sections5to8(const HwInputSample &in,
                         quat_t &outQw, quat_t &outQx,
                         quat_t &outQy, quat_t &outQz);

    // Direct BRAM helpers – addr is a 32-bit hardware word address
    float bram_read (sc_uint<32> addr);
    void  bram_write(sc_uint<32> addr, float value);

    // Internal quaternion state (fed back sample-to-sample within a batch)
    // Stored as sc_fixed<29,2> (quat_t): 27 fractional bits per analysis
    quat_t q0, q1, q2, q3;

    sc_core::sc_event start_event;
    sc_uint<32> batch_count;
};

#endif
