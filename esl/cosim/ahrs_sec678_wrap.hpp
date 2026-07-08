// =============================================================================
// ahrs_sec678_wrap.hpp
// SystemC wrapper (sc_foreign_module) za VHDL entity ahrs_sec678
//
// Portovi 1:1 sa entity ahrs_sec678 (ahrs_sec678.vhd):
//   clk, reset, start            -> std_logic
//   gx_i, gy_i, gz_i             -> signed(15:0)   Q9.7
//   hfbx_i, hfby_i, hfbz_i       -> signed(25:0)   Q2.24
//   gain_i                       -> unsigned(19:0) Q4.16
//   qw_i, qx_i, qy_i, qz_i       -> signed(17:0)   Q2.16
//   dt_i                         -> unsigned(19:0) Q0.20
//   ready                        -> std_logic
//   qw_o, qx_o, qy_o, qz_o       -> signed(17:0)   Q2.16
//   dbg_s1_hgx_o                 -> signed(21:0)
//   dbg_s2_adjgx_o               -> signed(25:0)
//   dbg_s3b_dqx_o                -> signed(26:0)
//   dbg_s4_shx_o                 -> signed(31:0)
//
// VHDL signed/unsigned mapiraju se na sc_lv<N> u SystemC wrapper-u.
// Konverzija float<->fixed se radi unutar AHRS_IP modula koji vozi signale.
// =============================================================================

#ifndef AHRS_SEC678_WRAP_HPP
#define AHRS_SEC678_WRAP_HPP

#include <systemc>

class ahrs_sec678_wrap : public sc_core::sc_foreign_module
{
public:
    // ── Clock / control ─────────────────────────────────────────────────────
    sc_core::sc_in<bool>              clk;
    sc_core::sc_in<sc_dt::sc_logic>   reset;
    sc_core::sc_in<sc_dt::sc_logic>   start;

    // ── Sec6: gyroscope Q9.7 ────────────────────────────────────────────────
    sc_core::sc_in<sc_dt::sc_lv<16> > gx_i;
    sc_core::sc_in<sc_dt::sc_lv<16> > gy_i;
    sc_core::sc_in<sc_dt::sc_lv<16> > gz_i;

    // ── Sec5 feedback Q2.24 ────────────────────────────────────────────────
    sc_core::sc_in<sc_dt::sc_lv<26> > hfbx_i;
    sc_core::sc_in<sc_dt::sc_lv<26> > hfby_i;
    sc_core::sc_in<sc_dt::sc_lv<26> > hfbz_i;

    // ── Sec7: gain Q4.16 ───────────────────────────────────────────────────
    sc_core::sc_in<sc_dt::sc_lv<20> > gain_i;

    // ── Sec8: input quaternion Q2.16 ───────────────────────────────────────
    sc_core::sc_in<sc_dt::sc_lv<18> > qw_i;
    sc_core::sc_in<sc_dt::sc_lv<18> > qx_i;
    sc_core::sc_in<sc_dt::sc_lv<18> > qy_i;
    sc_core::sc_in<sc_dt::sc_lv<18> > qz_i;

    // ── Sec8: delta time Q0.20 ─────────────────────────────────────────────
    sc_core::sc_in<sc_dt::sc_lv<20> > dt_i;

    // ── Outputs ────────────────────────────────────────────────────────────
    sc_core::sc_out<sc_dt::sc_logic>  ready;
    sc_core::sc_out<sc_dt::sc_lv<18> > qw_o;
    sc_core::sc_out<sc_dt::sc_lv<18> > qx_o;
    sc_core::sc_out<sc_dt::sc_lv<18> > qy_o;
    sc_core::sc_out<sc_dt::sc_lv<18> > qz_o;

    // ── DEBUG izlazi (vode se na signale ali se ne koriste obavezno) ───────
    sc_core::sc_out<sc_dt::sc_lv<22> > dbg_s1_hgx_o;
    sc_core::sc_out<sc_dt::sc_lv<26> > dbg_s2_adjgx_o;
    sc_core::sc_out<sc_dt::sc_lv<27> > dbg_s3b_dqx_o;
    sc_core::sc_out<sc_dt::sc_lv<32> > dbg_s4_shx_o;

    ahrs_sec678_wrap(sc_core::sc_module_name name) :
        sc_core::sc_foreign_module(name),
        clk("clk"), reset("reset"), start("start"),
        gx_i("gx_i"), gy_i("gy_i"), gz_i("gz_i"),
        hfbx_i("hfbx_i"), hfby_i("hfby_i"), hfbz_i("hfbz_i"),
        gain_i("gain_i"),
        qw_i("qw_i"), qx_i("qx_i"), qy_i("qy_i"), qz_i("qz_i"),
        dt_i("dt_i"),
        ready("ready"),
        qw_o("qw_o"), qx_o("qx_o"), qy_o("qy_o"), qz_o("qz_o"),
        dbg_s1_hgx_o("dbg_s1_hgx_o"),
        dbg_s2_adjgx_o("dbg_s2_adjgx_o"),
        dbg_s3b_dqx_o("dbg_s3b_dqx_o"),
        dbg_s4_shx_o("dbg_s4_shx_o")
    {
    }

    // Ime VHDL entiteta - Xcelium ce mapirati ovu klasu na taj entitet.
    const char* hdl_name() const { return "ahrs_sec678"; }
};

#endif // AHRS_SEC678_WRAP_HPP
