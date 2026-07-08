#include "ahrs_ip.hpp"

#ifdef COSIM
#include <cstdint>
#include <cstring>

// ============================================================================
// Pomocne funkcije za float <-> fixed-point konverziju
// (identicne semantici onih iz Vitis main.c, ali sa eksplicitnim maskama)
// ============================================================================

// gyro Q9.7 (signed 16-bit)
static inline uint32_t flt_to_q9_7(float f) {
    if (f >  255.99f) f =  255.99f;
    if (f < -256.0f)  f = -256.0f;
    return (uint32_t)((int32_t)(f * 128.0f) & 0xFFFF);
}

// gain Q4.16 (unsigned 20-bit)
static inline uint32_t flt_to_q4_16(float f) {
    if (f < 0.0f) f = 0.0f;
    return (uint32_t)((int32_t)(f * 65536.0f) & 0xFFFFF);
}

// dt Q0.20 (unsigned 20-bit)
static inline uint32_t flt_to_q0_20(float f) {
    if (f < 0.0f) f = 0.0f;
    return (uint32_t)((int32_t)(f * 1048576.0f) & 0xFFFFF);
}

// quaternion Q2.16 (signed 18-bit)
static inline uint32_t flt_to_q2_16(float f) {
    if (f >  1.99998f) f =  1.99998f;
    if (f < -2.0f)     f = -2.0f;
    return (uint32_t)((int32_t)(f * 65536.0f) & 0x3FFFF);
}

// hfb Q2.24 (signed 26-bit)
static inline uint32_t flt_to_q2_24(float f) {
    if (f >  1.99999994f) f =  1.99999994f;
    if (f < -2.0f)        f = -2.0f;
    return (uint32_t)((int32_t)(f * 16777216.0f) & 0x3FFFFFF);
}

// Q2.16 18-bit -> float (sign-extend pre cast-a)
static inline float q2_16_18b_to_flt(uint32_t u) {
    u &= 0x3FFFF;
    int32_t s = (u & 0x20000) ? (int32_t)(u | 0xFFFC0000U) : (int32_t)u;
    return (float)s / 65536.0f;
}
#endif // COSIM


// ============================================================================
// Constructor
// ============================================================================
AHRS_IP::AHRS_IP(sc_core::sc_module_name name)
    : sc_core::sc_module(name),
      tsock_interconnect("tsock_interconnect"),
      isock_bram("isock_bram")
#ifdef COSIM
    , m_clk(nullptr), m_dut(nullptr),
      sig_reset("sig_reset"), sig_start("sig_start"),
      sig_gx_i("sig_gx_i"), sig_gy_i("sig_gy_i"), sig_gz_i("sig_gz_i"),
      sig_hfbx_i("sig_hfbx_i"), sig_hfby_i("sig_hfby_i"), sig_hfbz_i("sig_hfbz_i"),
      sig_gain_i("sig_gain_i"),
      sig_qw_i("sig_qw_i"), sig_qx_i("sig_qx_i"),
      sig_qy_i("sig_qy_i"), sig_qz_i("sig_qz_i"),
      sig_dt_i("sig_dt_i"),
      sig_ready("sig_ready"),
      sig_qw_o("sig_qw_o"), sig_qx_o("sig_qx_o"),
      sig_qy_o("sig_qy_o"), sig_qz_o("sig_qz_o"),
      sig_dbg_s1_hgx("sig_dbg_s1_hgx"),
      sig_dbg_s2_adjgx("sig_dbg_s2_adjgx"),
      sig_dbg_s3b_dqx("sig_dbg_s3b_dqx"),
      sig_dbg_s4_shx("sig_dbg_s4_shx")
#endif
{
    tsock_interconnect.register_b_transport(this, &AHRS_IP::b_transport);
    SC_THREAD(solve);

    sample_count = (sc_uint<32>)0;

#ifdef COSIM
    // Sat: 100 MHz = 10ns period, identicno realnom HW (S00_AXI dizajn radi na 100MHz)
    m_clk = new sc_core::sc_clock("hdl_clk", 10, sc_core::SC_NS);

    // Instanciraj wrapper i poveži signale 1:1 sa portovima
    m_dut = new ahrs_sec678_wrap("dut");

    m_dut->clk(*m_clk);
    m_dut->reset(sig_reset);
    m_dut->start(sig_start);

    m_dut->gx_i(sig_gx_i); m_dut->gy_i(sig_gy_i); m_dut->gz_i(sig_gz_i);
    m_dut->hfbx_i(sig_hfbx_i); m_dut->hfby_i(sig_hfby_i); m_dut->hfbz_i(sig_hfbz_i);
    m_dut->gain_i(sig_gain_i);
    m_dut->qw_i(sig_qw_i); m_dut->qx_i(sig_qx_i);
    m_dut->qy_i(sig_qy_i); m_dut->qz_i(sig_qz_i);
    m_dut->dt_i(sig_dt_i);

    m_dut->ready(sig_ready);
    m_dut->qw_o(sig_qw_o); m_dut->qx_o(sig_qx_o);
    m_dut->qy_o(sig_qy_o); m_dut->qz_o(sig_qz_o);

    m_dut->dbg_s1_hgx_o(sig_dbg_s1_hgx);
    m_dut->dbg_s2_adjgx_o(sig_dbg_s2_adjgx);
    m_dut->dbg_s3b_dqx_o(sig_dbg_s3b_dqx);
    m_dut->dbg_s4_shx_o(sig_dbg_s4_shx);

    // Inicijalne vrednosti
    sig_reset.write(sc_dt::SC_LOGIC_0);
    sig_start.write(sc_dt::SC_LOGIC_0);
    sig_gx_i.write(sc_dt::sc_lv<16>(0));
    sig_gy_i.write(sc_dt::sc_lv<16>(0));
    sig_gz_i.write(sc_dt::sc_lv<16>(0));
    sig_hfbx_i.write(sc_dt::sc_lv<26>(0));
    sig_hfby_i.write(sc_dt::sc_lv<26>(0));
    sig_hfbz_i.write(sc_dt::sc_lv<26>(0));
    sig_gain_i.write(sc_dt::sc_lv<20>(0));
    sig_qw_i.write(sc_dt::sc_lv<18>(0));
    sig_qx_i.write(sc_dt::sc_lv<18>(0));
    sig_qy_i.write(sc_dt::sc_lv<18>(0));
    sig_qz_i.write(sc_dt::sc_lv<18>(0));
    sig_dt_i.write(sc_dt::sc_lv<20>(0));

    std::cout << "[AHRS_IP] Initialized. COSIM mode: HDL ahrs_sec678 instanciran. "
                 "100MHz clock, Sec5 u SystemC." << std::endl;
#else
    std::cout << "[AHRS_IP] Initialized. HW Sections 5-8 ready. Sample-by-sample mode." << std::endl;
#endif
}

AHRS_IP::~AHRS_IP() {
#ifdef COSIM
    delete m_dut;
    delete m_clk;
#endif
}

// ============================================================================
// b_transport
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
// ============================================================================
void AHRS_IP::solve()
{
#ifdef COSIM
    // Inicijalni reset HDL DUT-a
    wait(m_clk->negedge_event());
    sig_reset.write(sc_dt::SC_LOGIC_1);
    for (int i = 0; i < 4; i++) wait(m_clk->posedge_event());
    wait(m_clk->negedge_event());
    sig_reset.write(sc_dt::SC_LOGIC_0);
    wait(m_clk->posedge_event());
#endif

    while (true) {
        wait(start_event);
        sample_count++;

        // Procitaj jedan HwInputSample iz BRAM-a (uvek offset 0)
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

        float outQw_f, outQx_f, outQy_f, outQz_f;

#ifndef COSIM
        // ── Pure SystemC putanja ───────────────────────────────────────────
        quat_t outQw, outQx, outQy, outQz;
        hw_sections5to8(in, outQw, outQx, outQy, outQz);
        outQw_f = (float)outQw;  outQx_f = (float)outQx;
        outQy_f = (float)outQy;  outQz_f = (float)outQz;
#else
        // ── COSIM putanja: Sec5 u SC + HDL ahrs_sec678 ─────────────────────
        // 1) Sec5 u softveru -> hfbx/y/z kao float
        float hfbx, hfby, hfbz;
        sw_section5(in, hfbx, hfby, hfbz);

        // 2) Konvertuj sve ulaze u fixed-point bit pattern-e
        uint32_t gx_b   = flt_to_q9_7 (in.gx);
        uint32_t gy_b   = flt_to_q9_7 (in.gy);
        uint32_t gz_b   = flt_to_q9_7 (in.gz);
        uint32_t hfbx_b = flt_to_q2_24(hfbx);
        uint32_t hfby_b = flt_to_q2_24(hfby);
        uint32_t hfbz_b = flt_to_q2_24(hfbz);
        uint32_t gain_b = flt_to_q4_16(in.rampedGain);
        uint32_t qw_b   = flt_to_q2_16(in.qw);
        uint32_t qx_b   = flt_to_q2_16(in.qx);
        uint32_t qy_b   = flt_to_q2_16(in.qy);
        uint32_t qz_b   = flt_to_q2_16(in.qz);
        uint32_t dt_b   = flt_to_q0_20(in.deltaTime);

        // 3) Postavi signale na falling edge (da bi se setovali pre rising edge)
        wait(m_clk->negedge_event());

        sig_gx_i.write(sc_dt::sc_lv<16>(gx_b));
        sig_gy_i.write(sc_dt::sc_lv<16>(gy_b));
        sig_gz_i.write(sc_dt::sc_lv<16>(gz_b));
        sig_hfbx_i.write(sc_dt::sc_lv<26>(hfbx_b));
        sig_hfby_i.write(sc_dt::sc_lv<26>(hfby_b));
        sig_hfbz_i.write(sc_dt::sc_lv<26>(hfbz_b));
        sig_gain_i.write(sc_dt::sc_lv<20>(gain_b));
        sig_qw_i.write(sc_dt::sc_lv<18>(qw_b));
        sig_qx_i.write(sc_dt::sc_lv<18>(qx_b));
        sig_qy_i.write(sc_dt::sc_lv<18>(qy_b));
        sig_qz_i.write(sc_dt::sc_lv<18>(qz_b));
        sig_dt_i.write(sc_dt::sc_lv<20>(dt_b));

        sig_start.write(sc_dt::SC_LOGIC_1);

        // 4) Cekaj jedan rising edge -> HDL je uzorkovao start='1'
        wait(m_clk->posedge_event());

        // 5) Spusti start
        wait(m_clk->negedge_event());
        sig_start.write(sc_dt::SC_LOGIC_0);

        // 6) Cekaj ready (pipeline = 5 ciklusa, sa timeout-om za zastitu)
        int timeout = 100;
        while (sig_ready.read() != sc_dt::SC_LOGIC_1 && timeout > 0) {
            wait(m_clk->posedge_event());
            timeout--;
        }
        if (timeout <= 0) {
            std::cerr << "[AHRS_IP COSIM] TIMEOUT cekanja ready! sample="
                      << sample_count << std::endl;
        }

        // 7) Procitaj izlaze i konvertuj nazad u float
        outQw_f = q2_16_18b_to_flt(sig_qw_o.read().to_uint());
        outQx_f = q2_16_18b_to_flt(sig_qx_o.read().to_uint());
        outQy_f = q2_16_18b_to_flt(sig_qy_o.read().to_uint());
        outQz_f = q2_16_18b_to_flt(sig_qz_o.read().to_uint());

        #ifdef DEBUG_IP
        std::cout << "[AHRS_IP COSIM] sample=" << sample_count
                  << " dbg_s1_hgx=0x" << std::hex << sig_dbg_s1_hgx.read().to_uint()
                  << " dbg_s2_adjgx=0x" << sig_dbg_s2_adjgx.read().to_uint()
                  << " dbg_s3b_dqx=0x" << sig_dbg_s3b_dqx.read().to_uint()
                  << " dbg_s4_shx=0x" << sig_dbg_s4_shx.read().to_uint()
                  << std::dec << std::endl;
        #endif
#endif // COSIM

        // Upisi nenormalizovani kvaternion u BRAM
        bram_write(BRAM_OUTPUT_OFFSET + 0, outQw_f);
        bram_write(BRAM_OUTPUT_OFFSET + 1, outQx_f);
        bram_write(BRAM_OUTPUT_OFFSET + 2, outQy_f);
        bram_write(BRAM_OUTPUT_OFFSET + 3, outQz_f);

#ifndef COSIM
        // U SC rezimu simuliramo latenciju HW-a (65 ciklusa x 15ns)
        wait(AHRS_IP_LATENCY_NS, sc_core::SC_NS);
#endif

        ahrs_done.notify();
    }
}

#ifndef COSIM
// ============================================================================
// hw_sections5to8 – HW algoritam u sc_fixed aritmetici (V3 Final)
// (ORIGINAL - bez izmena)
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

    // SEKCIJA 7
    const gain_t rampedGain = in.rampedGain;
    const adjhg_t adjHalfGx = halfGx + halfFeedbackX * rampedGain;
    const adjhg_t adjHalfGy = halfGy + halfFeedbackY * rampedGain;
    const adjhg_t adjHalfGz = halfGz + halfFeedbackZ * rampedGain;

    // SEKCIJA 8
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
#endif // !COSIM

#ifdef COSIM
// ============================================================================
// sw_section5 - racun Sekcije 5 u SystemC (float aritmetika)
// Vraca halfFeedback*  kao float; AHRS_IP::solve() ce ga konvertovati u Q2.24
// ============================================================================
static inline float fast_inv_sqrt_cosim(float x)
{
    union { float f; int32_t i; } u = {x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

void AHRS_IP::sw_section5(const HwInputSample &in,
                          float &hfbx, float &hfby, float &hfbz)
{
    hfbx = 0.0f; hfby = 0.0f; hfbz = 0.0f;

    const bool accelNotZero = !((in.ax == 0.0f) && (in.ay == 0.0f) && (in.az == 0.0f));
    if (!accelNotZero) return;

    const float ax = in.ax, ay = in.ay, az = in.az;
    const float hgX = in.halfGravityX, hgY = in.halfGravityY, hgZ = in.halfGravityZ;

    const float magSq  = ax*ax + ay*ay + az*az;
    const float invMag = fast_inv_sqrt_cosim(magSq);

    float nx = ax * invMag;
    float ny = ay * invMag;
    float nz = az * invMag;

    float cx = ny * hgZ - nz * hgY;
    float cy = nz * hgX - nx * hgZ;
    float cz = nx * hgY - ny * hgX;

    const float dot = nx*hgX + ny*hgY + nz*hgZ;
    if (dot < 0.0f) {
        const float csq = cx*cx + cy*cy + cz*cz;
        const float ci  = fast_inv_sqrt_cosim(csq);
        cx *= ci; cy *= ci; cz *= ci;
    }

    hfbx = cx; hfby = cy; hfbz = cz;
}
#endif // COSIM


// ============================================================================
// BRAM helpers (ORIGINAL)
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
