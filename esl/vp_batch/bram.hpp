#ifndef BRAM_HPP
#define BRAM_HPP

#include "defines.hpp"

/**
 * @brief Dual-port BRAM for AHRS system
 * 
 * Port A: CPU access (read/write control registers, input/output buffers)
 * Port B: AHRS IP access (read inputs, write outputs)
 * 
 * Memory layout:
 *   0x0000-0x000F: Control/Status registers (16 words)
 *   0x0010-0x001F: AHRS state (quaternion, gains) (16 words)
 *   0x0020-0x00CF: Input buffer - HwInputSample[10] (176 words)
 *   0x00D0-0x010F: Output buffer - HwOutputSample[10] (64 words)
 *   0x0110-0x01FF: Reserved/Debug (240 words)
 */
class Bram : public sc_core::sc_module {

public:
    Bram(sc_core::sc_module_name name);
    
    tlm_utils::simple_target_socket<Bram> tsock_a;  // Port A: CPU
    tlm_utils::simple_target_socket<Bram> tsock_b;  // Port B: AHRS IP

    void b_transport(tlm::tlm_generic_payload&, sc_core::sc_time&);
    
private:
    // BRAM storage: 512 words × 32-bit (IEEE 754 float bit patterns)
    uint32_t mem[BRAM_SIZE];
};

#endif