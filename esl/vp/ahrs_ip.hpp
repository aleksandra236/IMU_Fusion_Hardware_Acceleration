#ifndef AHRS_IP_HPP
#define AHRS_IP_HPP

#include "defines.hpp"
#include <cmath>

// ============================================================================
// Fixed-Point Type Aliases (from fixed-point analysis – V3 Final)
// ============================================================================
typedef sc_fixed<18, 2, SC_RND, SC_SAT>  quat_t;      // kvaternion [-2,2),  16 frac bita
typedef sc_fixed<26, 1, SC_RND, SC_SAT>  dot_t;       // skalarni pr. [-1,1), 25 frac bita
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  norm_t;      // norm. accel / cross, 24 frac bita
typedef sc_fixed<20, 3, SC_RND, SC_SAT>  accel_t;     // accel ulaz [-4,4), 17 frac bita
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  hg_t;        // halfGravity [-2,2), 24 frac bita
typedef sc_ufixed<22, 2, SC_RND, SC_SAT> magsq_t;     // magnituda²  [0,4),  20 frac bita
typedef sc_ufixed<24, 6, SC_RND, SC_SAT> invmag_t;    // inv-mag     [0,64), 18 frac bita
typedef sc_fixed<22, 2, SC_RND, SC_SAT>  halfgyro_t;  // polu-ziro  [-2,2), 20 frac bita
typedef sc_ufixed<20, 4, SC_RND, SC_SAT> gain_t;      // gain        [0,16), 16 frac bita
typedef sc_fixed<26, 5, SC_RND, SC_SAT>  adjhg_t;     // adj polu-ziro, 21 frac bita
typedef sc_fixed<27, 5, SC_RND, SC_SAT>  dq_t;        // d-kvaternion, 22 frac bita
typedef sc_ufixed<20, 0, SC_RND, SC_SAT> dt_t;        // deltaTime   [0,1),  20 frac bita

/**
 * @brief AHRS Hardware Accelerator (IP core) – Sekcije 5-8
 *
 * Sample-by-sample rezim: jedan start_event = jedan uzorak.
 * Kvaternion stanje NE cuva se interno – CPU salje normalizovani q
 * (iz SW Sekcije 9) kroz BRAM na BRAM_INPUT_OFFSET+11..14 pri svakom pozivu.
 * IP cita taj q, procesira sekcije 5-8 i vraca nenormalizovani q u BRAM.
 *
 * Socket raspored:
 *   tsock_interconnect ← Interconnect::isock_ip   (kontrolni upisi od CPU)
 *   isock_bram         → Bram::tsock_b             (direktan pristup podacima)
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

    void hw_sections5to8(const HwInputSample &in,
                         quat_t &outQw, quat_t &outQx,
                         quat_t &outQy, quat_t &outQz);

    float bram_read (sc_uint<32> addr);
    void  bram_write(sc_uint<32> addr, float value);

    // Napomena: q stanje se NE cuva interno. CPU salje normalizovani q
    // kroz BRAM (word offsets 11-14) pri svakom uzorku.

    sc_core::sc_event start_event;
    sc_uint<32> sample_count;  // brojac uzoraka (za statistiku)
};

#endif