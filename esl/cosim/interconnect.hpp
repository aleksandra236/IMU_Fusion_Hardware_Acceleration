#ifndef INTERCONNECT_HPP
#define INTERCONNECT_HPP

#include "defines.hpp"

/**
 * @brief TLM Interconnect for AHRS system
 * 
 * Routes transactions from CPU to:
 *   - BRAM (0x0000 - 0x01FF): Data storage
 *   - AHRS IP (0x0200 - 0x020F): Control registers
 * 
 * Memory Map:
 *   0x0000-0x01FF → BRAM
 *   0x0200-0x020F → AHRS IP
 */
class Interconnect : public sc_core::sc_module {

public:
    Interconnect(sc_core::sc_module_name name);

    tlm_utils::simple_target_socket<Interconnect>    tsock_cpu;
    tlm_utils::simple_initiator_socket<Interconnect> isock_bram;
    tlm_utils::simple_initiator_socket<Interconnect> isock_ip;
    tlm_utils::simple_initiator_socket<Interconnect> isock_spi;

    void b_transport(tlm::tlm_generic_payload&, sc_core::sc_time&);
    
private:
    void print_transaction(const char* destination, uint32_t addr, tlm::tlm_command cmd);
};

#endif