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

    sample_count = (sc_uint<32>)0;

    std::cout << "[AHRS_IP] Initialized. HW Sections 5-8 ready. Sample-by-sample mode." << std::endl;
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
        float status = (sample_count > 0) ? 2.0f : 0.0f;
        *reinterpret_cast<float *>(buf) = status;
        pl.set_response_status(tlm::TLM_OK_RESPONSE);
        wait(10, sc_core::SC_NS);
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
            start_event.notify();
        }
    }

    pl.set_response_status(tlm::TLM_OK_RESPONSE);
    wait(10, sc_core::SC_NS);
}

// ============================================================================
// solve – main SC_THREAD, sample-by-sample
// Svaki start_event = jedan uzorak
// ============================================================================
void AHRS_IP::solve()
{
    while (true) {
        wait(start_event);
        sample_count++;

        // Citaj jedan HwInputSample iz BRAM-a (uvek na offset 0)
        HwInputSample in;
        in.gx           = bram_read(BRAM_INPUT_OFFSET + 0);
        in.gy           = bram_read(BRAM_INPUT_OFFSET + 1);
        in.gz           = bram_read(BRAM_INPUT_OFFSET + 2);
        in.ax           = bram_read(BRAM_INPUT_OFFSET + 3);
        in.ay           = bram_read(BRAM_INPUT_OFFSET + 4);
        in.az           = bram_read(BRAM_INPUT_OFFSET + 5);
        in.halfGravityX = bram_read(BRAM_INPUT_OFFSET + 6);
        in.halfGravityY = bram_read(BRAM_INPUT_OFFSET + 7);
        in.halfGravityZ = bram_read(BRAM_INPUT_OFFSET + 8);
        in.deltaTime    = bram_read(BRAM_INPUT_OFFSET + 9);
        in.rampedGain   = bram_read(BRAM_INPUT_OFFSET + 10);
        in.qw           = bram_read(BRAM_INPUT_OFFSET + 11);
        in.qx           = bram_read(BRAM_INPUT_OFFSET + 12);
        in.qy           = bram_read(BRAM_INPUT_OFFSET + 13);
        in.qz           = bram_read(BRAM_INPUT_OFFSET + 14);

        // Izvrsi HW Sekcije 5-8 za jedan uzorak
        quat_t outQw, outQx, outQy, outQz;
        hw_sections5to8(in, outQw, outQx, outQy, outQz);

        // Upisi nenormalizovani kvaternion u BRAM (uvek na offset 0)
        bram_write(BRAM_OUTPUT_OFFSET + 0, (float)outQw);
        bram_write(BRAM_OUTPUT_OFFSET + 1, (float)outQx);
        bram_write(BRAM_OUTPUT_OFFSET + 2, (float)outQy);
        bram_write(BRAM_OUTPUT_OFFSET + 3, (float)outQz);

        // Simulacija hardverske latencije: 65 ciklusa × 15ns = 975ns
        wait(AHRS_IP_LATENCY_NS, sc_core::SC_NS);

        // Obavesti CPU
        ahrs_done.notify();
    }
}

// ============================================================================
// hw_sections5to8 – HW algoritam u sc_fixed aritmetici (V3 Final tipovi)
// ============================================================================
static inline float fast_inv_sqrt_ip(float x)
{
    union { float f; int32_t i; } u = {x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

void AHRS_IP::hw_sections5to8(const HwInputSample &in,
                               quat_t &outQw, quat_t &outQx,
                               quat_t &outQy, quat_t &outQz)
{
    // SEKCIJA 5: Povratna sprega akcelerometra
    norm_t halfFeedbackX = 0.0f, halfFeedbackY = 0.0f, halfFeedbackZ = 0.0f;

    const bool accelNotZero(!((in.ax == 0.0f) && (in.ay == 0.0f) && (in.az == 0.0f)));
    if (accelNotZero) {
        const accel_t ax_fp = in.ax;
        const accel_t ay_fp = in.ay;
        const accel_t az_fp = in.az;

        const hg_t hgX = in.halfGravityX;
        const hg_t hgY = in.halfGravityY;
        const hg_t hgZ = in.halfGravityZ;

        const magsq_t accelMagSq = ax_fp * ax_fp + ay_fp * ay_fp + az_fp * az_fp;
        const invmag_t accelInvMag = fast_inv_sqrt_ip((float)accelMagSq);

        const norm_t ax = ax_fp * accelInvMag;
        const norm_t ay = ay_fp * accelInvMag;
        const norm_t az = az_fp * accelInvMag;

        norm_t crossX = ay * hgZ - az * hgY;
        norm_t crossY = az * hgX - ax * hgZ;
        norm_t crossZ = ax * hgY - ay * hgX;

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

    // SEKCIJA 6: Ziroskop deg/s → polu-ugaona brzina
    const float DEG_TO_HALF_RAD = 3.14159265358979f / 360.0f;
    const halfgyro_t halfGx = in.gx * DEG_TO_HALF_RAD;
    const halfgyro_t halfGy = in.gy * DEG_TO_HALF_RAD;
    const halfgyro_t halfGz = in.gz * DEG_TO_HALF_RAD;

    // SEKCIJA 7: Primena povratne sprege
    const gain_t rampedGain = in.rampedGain;
    const adjhg_t adjHalfGx = halfGx + halfFeedbackX * rampedGain;
    const adjhg_t adjHalfGy = halfGy + halfFeedbackY * rampedGain;
    const adjhg_t adjHalfGz = halfGz + halfFeedbackZ * rampedGain;

    // SEKCIJA 8: Integracija kvaterniona (prvi red)
    // Koristi normalizovani q od CPU-a (ne interno stanje) da bez divergencije
    const dt_t dt = in.deltaTime;
    const quat_t iq0 = in.qw, iq1 = in.qx, iq2 = in.qy, iq3 = in.qz;

    const dq_t dqw = -iq1 * adjHalfGx - iq2 * adjHalfGy - iq3 * adjHalfGz;
    const dq_t dqx =  iq0 * adjHalfGx + iq2 * adjHalfGz - iq3 * adjHalfGy;
    const dq_t dqy =  iq0 * adjHalfGy - iq1 * adjHalfGz + iq3 * adjHalfGx;
    const dq_t dqz =  iq0 * adjHalfGz + iq1 * adjHalfGy - iq2 * adjHalfGx;

    const quat_t resultQw = iq0 + dqw * dt;
    const quat_t resultQx = iq1 + dqx * dt;
    const quat_t resultQy = iq2 + dqy * dt;
    const quat_t resultQz = iq3 + dqz * dt;

    outQw = (float)resultQw;
    outQx = (float)resultQx;
    outQy = (float)resultQy;
    outQz = (float)resultQz;
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