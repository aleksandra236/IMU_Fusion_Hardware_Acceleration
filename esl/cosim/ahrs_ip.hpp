#ifndef AHRS_IP_HPP
#define AHRS_IP_HPP

#include "defines.hpp"
#include <cmath>

#ifdef COSIM
#include "ahrs_sec678_wrap.hpp"
#endif

// ============================================================================
// Fixed-Point Type Aliases (V3 Final) - koriste se SAMO u non-COSIM rezimu
// ============================================================================
#ifndef COSIM
typedef sc_fixed<18, 2, SC_RND, SC_SAT>  quat_t;
typedef sc_fixed<26, 1, SC_RND, SC_SAT>  dot_t;
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  norm_t;
typedef sc_fixed<20, 3, SC_RND, SC_SAT>  accel_t;
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  hg_t;
typedef sc_ufixed<22, 2, SC_RND, SC_SAT> magsq_t;
typedef sc_ufixed<24, 6, SC_RND, SC_SAT> invmag_t;
typedef sc_fixed<22, 2, SC_RND, SC_SAT>  halfgyro_t;
typedef sc_ufixed<20, 4, SC_RND, SC_SAT> gain_t;
typedef sc_fixed<26, 5, SC_RND, SC_SAT>  adjhg_t;
typedef sc_fixed<27, 5, SC_RND, SC_SAT>  dq_t;
typedef sc_ufixed<20, 0, SC_RND, SC_SAT> dt_t;
#endif

/**
 * @brief AHRS Hardware Accelerator (IP core) – Sekcije 5-8
 *
 * Dva rezima:
 *   1) SystemC referentni model (default) - hw_sections5to8 racuna u sc_fixed
 *   2) COSIM rezim (-DCOSIM) - racuna VHDL entity ahrs_sec678 instanciran
 *      preko sc_foreign_module wrapper-a. Sekcija 5 ostaje u SystemC i daje
 *      hfbx/y/z (Q2.24) koji se vode u HDL.
 */
class AHRS_IP : public sc_core::sc_module {
public:
    SC_HAS_PROCESS(AHRS_IP);
    AHRS_IP(sc_core::sc_module_name name);
    ~AHRS_IP();

    tlm_utils::simple_target_socket<AHRS_IP>    tsock_interconnect;
    tlm_utils::simple_initiator_socket<AHRS_IP> isock_bram;

    void b_transport(tlm::tlm_generic_payload&, sc_core::sc_time&);

private:
    void solve();

#ifndef COSIM
    // Pure-SystemC putanja (originalna implementacija)
    void hw_sections5to8(const HwInputSample &in,
                         quat_t &outQw, quat_t &outQx,
                         quat_t &outQy, quat_t &outQz);
#else
    // COSIM putanja - racuna samo Sekciju 5 u SC, ostalo radi HDL
    void sw_section5(const HwInputSample &in,
                     float &hfbx, float &hfby, float &hfbz);
#endif

    float bram_read (sc_uint<32> addr);
    void  bram_write(sc_uint<32> addr, float value);

    sc_core::sc_event start_event;
    sc_uint<32> sample_count;

#ifdef COSIM
    // ── HDL kosimulacija: sat, signali za drive-ovanje wrapper-a ───────────
    sc_core::sc_clock                    *m_clk;
    ahrs_sec678_wrap                     *m_dut;

    sc_core::sc_signal<sc_dt::sc_logic>   sig_reset;
    sc_core::sc_signal<sc_dt::sc_logic>   sig_start;

    sc_core::sc_signal<sc_dt::sc_lv<16> > sig_gx_i;
    sc_core::sc_signal<sc_dt::sc_lv<16> > sig_gy_i;
    sc_core::sc_signal<sc_dt::sc_lv<16> > sig_gz_i;

    sc_core::sc_signal<sc_dt::sc_lv<26> > sig_hfbx_i;
    sc_core::sc_signal<sc_dt::sc_lv<26> > sig_hfby_i;
    sc_core::sc_signal<sc_dt::sc_lv<26> > sig_hfbz_i;

    sc_core::sc_signal<sc_dt::sc_lv<20> > sig_gain_i;

    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qw_i;
    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qx_i;
    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qy_i;
    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qz_i;

    sc_core::sc_signal<sc_dt::sc_lv<20> > sig_dt_i;

    sc_core::sc_signal<sc_dt::sc_logic>   sig_ready;
    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qw_o;
    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qx_o;
    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qy_o;
    sc_core::sc_signal<sc_dt::sc_lv<18> > sig_qz_o;

    sc_core::sc_signal<sc_dt::sc_lv<22> > sig_dbg_s1_hgx;
    sc_core::sc_signal<sc_dt::sc_lv<26> > sig_dbg_s2_adjgx;
    sc_core::sc_signal<sc_dt::sc_lv<27> > sig_dbg_s3b_dqx;
    sc_core::sc_signal<sc_dt::sc_lv<32> > sig_dbg_s4_shx;
#endif
};

#endif // AHRS_IP_HPP
