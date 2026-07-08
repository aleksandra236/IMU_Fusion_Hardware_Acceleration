#include "ahrs_ip.hpp"

// ============================================================================
// Constructor
// ============================================================================
AHRS_IP::AHRS_IP(sc_core::sc_module_name name)
    : sc_core::sc_module(name),
      tsock_interconnect("tsock_interconnect"),
      isock_bram("isock_bram")
{
    tsock_interconnect.register_b_transport(this, &AHRS_IP::b_transport);
    SC_THREAD(solve);

    // Identity quaternion – CPU will have already set up initial state via BRAM
    q0 = 1.0f;  q1 = 0.0f;  q2 = 0.0f;  q3 = 0.0f;

    batch_count = (sc_uint<32>)0;

    std::cout << "[AHRS_IP] Initialized. HW Sections 5-8 ready." << std::endl;
}

// ============================================================================
// b_transport – control register writes from CPU (START signal)
// ============================================================================
void AHRS_IP::b_transport(tlm::tlm_generic_payload &pl, sc_core::sc_time &offset)
{
    (void)offset;

    sc_uint<32>    addr = (sc_uint<32>)pl.get_address();
    unsigned char *buf  = pl.get_data_ptr();
    tlm::tlm_command cmd = pl.get_command();

    if (cmd == tlm::TLM_READ_COMMAND) {
        // Status register read
        float status = (batch_count > 0) ? 2.0f : 0.0f;  // bit1=done
        *reinterpret_cast<float *>(buf) = status;
        pl.set_response_status(tlm::TLM_OK_RESPONSE);
        wait(10, sc_core::SC_NS);  // 1 clock cycle @ 10 ns
        return;
    }

    if (cmd != tlm::TLM_WRITE_COMMAND) {
        pl.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
        return;
    }

    sc_uint<32> reg = addr - IP_ADDR_OFFSET;
    float       val = *reinterpret_cast<float *>(buf);

    if (reg == IP_REG_CTRL) {
        sc_uint<32> val_bits = (sc_uint<32>)(int)val;
        if (val_bits[CTRL_BIT_START]) {
            std::cout << "[AHRS_IP] Start signal received from CPU." << std::endl;
            start_event.notify();
        }
    }

    pl.set_response_status(tlm::TLM_OK_RESPONSE);
    wait(10, sc_core::SC_NS);  // 1 clock cycle @ 10 ns
}

// ============================================================================
// solve – main SC_THREAD, wakes on start_event
// ============================================================================
void AHRS_IP::solve()
{
    while (true) {
        wait(start_event);
        batch_count++;

        std::cout << "[AHRS_IP] Processing batch " << batch_count
                  << " (Sections 5-8)..." << std::endl;

        for (int i = 0; i < HW_BATCH_SIZE; i++) {

            // ── Read HwInputSample from BRAM input buffer ────────────────────
            sc_uint<32> in_addr = (sc_uint<32>)(BRAM_INPUT_OFFSET + i * INPUT_SAMPLE_WORDS);

            HwInputSample in;
            in.gx           = bram_read(in_addr + 0);
            in.gy           = bram_read(in_addr + 1);
            in.gz           = bram_read(in_addr + 2);
            in.ax           = bram_read(in_addr + 3);
            in.ay           = bram_read(in_addr + 4);
            in.az           = bram_read(in_addr + 5);
            in.halfGravityX = bram_read(in_addr + 6);
            in.halfGravityY = bram_read(in_addr + 7);
            in.halfGravityZ = bram_read(in_addr + 8);
            in.deltaTime    = bram_read(in_addr + 9);
            in.rampedGain   = bram_read(in_addr + 10);

            // ── Run HW Sections 5-8 ──────────────────────────────────────────
            quat_t outQw, outQx, outQy, outQz;
            hw_sections5to8(in, outQw, outQx, outQy, outQz);

            // ── Write UNNORMALIZED output quaternion to BRAM output buffer ───
            sc_uint<32> out_addr = (sc_uint<32>)(BRAM_OUTPUT_OFFSET + i * OUTPUT_SAMPLE_WORDS);
            bram_write(out_addr + 0, (float)outQw);
            bram_write(out_addr + 1, (float)outQx);
            bram_write(out_addr + 2, (float)outQy);
            bram_write(out_addr + 3, (float)outQz);

            // Simulate hardware clock cycles: 1 µs per sample
            wait(AHRS_IP_LATENCY_NS, sc_core::SC_NS);
        }

        std::cout << "[AHRS_IP] Batch " << batch_count
                  << " done. Internal q=("
                  << (float)q0 << ", " << (float)q1 << ", "
                  << (float)q2 << ", " << (float)q3 << ")"
                  << " [unnormalized]" << std::endl;

        // Fire interrupt → CPU
        ahrs_done.notify();
    }
}

// Fast inverse square-root – identical bit-magic to spec ahrs_hw.cpp
static inline float fast_inv_sqrt_ip(float x)
{
    union { float f; int32_t i; } u = {x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

// ============================================================================
// hw_sections5to8 – HW algorithm in sc_fixed arithmetic (V3 Final types)
//
// Section 5: Accelerometer feedback using halfGravity (from SW Sec.4)
// Section 6: Gyroscope deg/s → half-angle rad/s
// Section 7: Apply feedback to gyroscope rates (× rampedGain)
// Section 8: First-order quaternion integration → UNNORMALIZED output
//
// All internal computation uses sc_fixed types derived from fixed-point
// analysis (analysis/fixed_point_analysis.md, V3 Final configuration).
// Inputs arrive as float from BRAM; outputs are cast back to float for BRAM.
// ============================================================================
void AHRS_IP::hw_sections5to8(const HwInputSample &in,
                               quat_t &outQw, quat_t &outQx,
                               quat_t &outQy, quat_t &outQz)
{
    // ── SECTION 5: Accelerometer feedback ───────────────────────────────────
    norm_t halfFeedbackX = 0.0f, halfFeedbackY = 0.0f, halfFeedbackZ = 0.0f;

    const bool accelNotZero(!((in.ax == 0.0f) && (in.ay == 0.0f) && (in.az == 0.0f)));
    if (accelNotZero) {
        // Convert accel inputs to fixed-point (accel_t: 20-bit, 17 frac)
        const accel_t ax_fp = in.ax;
        const accel_t ay_fp = in.ay;
        const accel_t az_fp = in.az;

        // Convert halfGravity to fixed-point (hg_t: 26-bit, 24 frac)
        const hg_t hgX = in.halfGravityX;
        const hg_t hgY = in.halfGravityY;
        const hg_t hgZ = in.halfGravityZ;

        // FAST INV SQRT #1 – accelerometer normalisation
        // fast_inv_sqrt uses float bit-trick; result cast to invmag_t
        const magsq_t accelMagSq = ax_fp * ax_fp + ay_fp * ay_fp + az_fp * az_fp;
        const invmag_t accelInvMag = fast_inv_sqrt_ip((float)accelMagSq);

        // Normalised accelerometer (norm_t: 26-bit, 24 frac)
        const norm_t ax = ax_fp * accelInvMag;
        const norm_t ay = ay_fp * accelInvMag;
        const norm_t az = az_fp * accelInvMag;

        // Cross product: normAccel × halfGravity  (largest error source per analysis)
        norm_t crossX = ay * hgZ - az * hgY;
        norm_t crossY = az * hgX - ax * hgZ;
        norm_t crossZ = ax * hgY - ay * hgX;

        // Dot product check: if accel opposed to estimated gravity,
        // normalise cross product (FAST INV SQRT #2)
        const dot_t dot = ax * hgX + ay * hgY + az * hgZ;
        if (dot < 0.0f) {
            const magsq_t crossMagSq = crossX * crossX + crossY * crossY + crossZ * crossZ;
            const invmag_t crossInvMag = fast_inv_sqrt_ip((float)crossMagSq);
            crossX = crossX * crossInvMag;
            crossY = crossY * crossInvMag;
            crossZ = crossZ * crossInvMag;
        }

        halfFeedbackX = crossX;
        halfFeedbackY = crossY;
        halfFeedbackZ = crossZ;
    }

    // ── SECTION 6: Gyroscope deg/s → half-angle rad/s ───────────────────────
    const float DEG_TO_HALF_RAD = 3.14159265358979f / 360.0f;  // π/360
    const halfgyro_t halfGx = in.gx * DEG_TO_HALF_RAD;
    const halfgyro_t halfGy = in.gy * DEG_TO_HALF_RAD;
    const halfgyro_t halfGz = in.gz * DEG_TO_HALF_RAD;

    // ── SECTION 7: Apply proportional feedback (adjhg_t: 26-bit, 21 frac) ──
    const gain_t rampedGain = in.rampedGain;
    const adjhg_t adjHalfGx = halfGx + halfFeedbackX * rampedGain;
    const adjhg_t adjHalfGy = halfGy + halfFeedbackY * rampedGain;
    const adjhg_t adjHalfGz = halfGz + halfFeedbackZ * rampedGain;

    // ── SECTION 8: Quaternion integration (first-order) ─────────────────────
    //   q_new = q + q ⊗ [0, adjHalfG] × dt
    const dt_t dt = in.deltaTime;

    const dq_t dqw = -q1 * adjHalfGx - q2 * adjHalfGy - q3 * adjHalfGz;
    const dq_t dqx =  q0 * adjHalfGx + q2 * adjHalfGz - q3 * adjHalfGy;
    const dq_t dqy =  q0 * adjHalfGy - q1 * adjHalfGz + q3 * adjHalfGx;
    const dq_t dqz =  q0 * adjHalfGz + q1 * adjHalfGy - q2 * adjHalfGx;

    const quat_t resultQw = q0 + dqw * dt;
    const quat_t resultQx = q1 + dqx * dt;
    const quat_t resultQy = q2 + dqy * dt;
    const quat_t resultQz = q3 + dqz * dt;

    // Cast fixed-point result back to float for BRAM storage
    outQw = (float)resultQw;
    outQx = (float)resultQx;
    outQy = (float)resultQy;
    outQz = (float)resultQz;

    // Feed unnormalized sc_fixed result forward to next sample in this batch.
    // CPU will normalise after the whole batch (SW Sec.9).
    q0 = resultQw;
    q1 = resultQx;
    q2 = resultQy;
    q3 = resultQz;
}

// ============================================================================
// BRAM helpers
// ============================================================================
float AHRS_IP::bram_read(sc_uint<32> addr)
{
    tlm::tlm_generic_payload pl;
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME;
    float                    data  = 0.0f;

    pl.set_command(tlm::TLM_READ_COMMAND);
    pl.set_address((uint64_t)addr.to_uint());
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data));
    pl.set_data_length(1);
    pl.set_streaming_width(1);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_bram->b_transport(pl, delay);

    return data;
}

void AHRS_IP::bram_write(sc_uint<32> addr, float value)
{
    tlm::tlm_generic_payload pl;
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME;
    float                    data  = value;

    pl.set_command(tlm::TLM_WRITE_COMMAND);
    pl.set_address((uint64_t)addr.to_uint());
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data));
    pl.set_data_length(1);
    pl.set_streaming_width(1);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_bram->b_transport(pl, delay);
}
