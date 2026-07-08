#include "interconnect.hpp"

Interconnect::Interconnect(sc_core::sc_module_name name) : sc_core::sc_module(name), 
    tsock_cpu("tsock_cpu"),
    isock_bram("isock_bram"), 
    isock_ip("isock_ip"),
    isock_spi("isock_spi") {
    
    tsock_cpu.register_b_transport(this, &Interconnect::b_transport); // this tells, when CPU sends
    //a transaction to the Interconnect through tsock_cpu, the Interconnect's 
    //b_transport method should be called to handle that transaction.
    
    std::cout << "[INTERCONNECT] Initialized" << std::endl;
    std::cout << "  BRAM:    0x" << std::hex << BRAM_ADDR_OFFSET 
              << " - 0x" << BRAM_ADDR_MAX << std::dec << std::endl;
    std::cout << "  AHRS IP: 0x" << std::hex << IP_ADDR_OFFSET 
              << " - 0x" << (IP_ADDR_OFFSET + IP_ADDR_MAX) << std::dec << std::endl;
    std::cout << "  SPI:     0x" << std::hex << SPI_ADDR_OFFSET 
              << " - 0x" << (SPI_ADDR_OFFSET + SPI_ADDR_MAX) << std::dec << std::endl;
}

void Interconnect::b_transport(tlm::tlm_generic_payload &pl, sc_core::sc_time &offset){

    uint32_t addr = (uint32_t)pl.get_address();
    
    #ifdef DEBUG_INTERCONNECT
    tlm::tlm_command cmd = pl.get_command();
    #endif

    // Address decoding and routing
    if (addr <= BRAM_ADDR_MAX) {
        // Route to BRAM
        #ifdef DEBUG_INTERCONNECT
        print_transaction("BRAM", addr, cmd);
        #endif
        isock_bram->b_transport(pl, offset);
        
    } else if (addr >= IP_ADDR_OFFSET && addr <= (IP_ADDR_OFFSET + IP_ADDR_MAX)) {
        // Route to AHRS IP
        #ifdef DEBUG_INTERCONNECT
        print_transaction("AHRS_IP", addr, cmd);
        #endif
        isock_ip->b_transport(pl, offset);
        
    } else if (addr >= SPI_ADDR_OFFSET && addr <= (SPI_ADDR_OFFSET + SPI_ADDR_MAX)) {
        // Route to SPI Controller
        #ifdef DEBUG_INTERCONNECT
        print_transaction("SPI", addr, cmd);
        #endif
        pl.set_address(addr - SPI_ADDR_OFFSET);  // relative address for SPI
        isock_spi->b_transport(pl, offset);
        pl.set_address(addr);  // restore original address
        
    } else {
        // Invalid address
        std::cout << "[INTERCONNECT] Error: Invalid address 0x" 
                  << std::hex << addr << std::dec << std::endl;
        std::cout << "  Valid ranges:" << std::endl;
        std::cout << "    BRAM:    0x" << std::hex << BRAM_ADDR_OFFSET 
                  << " - 0x" << BRAM_ADDR_MAX << std::dec << std::endl;
        std::cout << "    AHRS IP: 0x" << std::hex << IP_ADDR_OFFSET 
                  << " - 0x" << (IP_ADDR_OFFSET + IP_ADDR_MAX) << std::dec << std::endl;
        std::cout << "    SPI:     0x" << std::hex << SPI_ADDR_OFFSET 
                  << " - 0x" << (SPI_ADDR_OFFSET + SPI_ADDR_MAX) << std::dec << std::endl;
        pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
    }

    // Interconnect routing delay: 1 clock cycle @ 10 ns
    offset += sc_core::sc_time(10, sc_core::SC_NS);
}

void Interconnect::print_transaction(const char* destination, uint32_t addr, tlm::tlm_command cmd) {
    const char* cmd_str = (cmd == tlm::TLM_READ_COMMAND) ? "READ" : "WRITE";
    std::cout << "[INTERCONNECT] Routing " << cmd_str << " to " << destination 
              << " @ 0x" << std::hex << addr << std::dec << std::endl;
}