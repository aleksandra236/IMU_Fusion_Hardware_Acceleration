#include "bram.hpp"

Bram::Bram(sc_core::sc_module_name name) : sc_core::sc_module(name), tsock_a("tsock_a"), tsock_b("tsock_b") {
    tsock_a.register_b_transport(this, &Bram::b_transport);
    tsock_b.register_b_transport(this, &Bram::b_transport);

    // Initialize BRAM to zeros
    for(int i = 0; i < BRAM_SIZE; i++){
        mem[i] = 0;
    }
    
    std::cout << "[BRAM] Initialized: " << BRAM_SIZE << " words × 32-bit (2 KB)" << std::endl;
}

void Bram::b_transport(tlm::tlm_generic_payload &pl, sc_core::sc_time &offset){

    uint32_t         addr     = (uint32_t)pl.get_address();
    uint32_t         len      = (uint32_t)pl.get_data_length();
    unsigned char   *buf      = pl.get_data_ptr();
    tlm::tlm_command cmd      = pl.get_command();
    
    (void)offset;  // Unused in current implementation

    // Address validation
    if (addr > BRAM_ADDR_MAX) {
        std::cout << "[BRAM] Error: Address out of range: 0x" << std::hex << addr << std::dec << std::endl;
        pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        return;
    }

    // Convert address to word index
    uint32_t word_addr = addr - BRAM_ADDR_OFFSET;
    
    switch(cmd){
        case tlm::TLM_READ_COMMAND:
            // Read 'len' words from BRAM; interpret stored uint32_t bits as float
            for(uint32_t i = 0; i < len; i++){
                if (word_addr + i < BRAM_SIZE) {
                    reinterpret_cast<float*>(buf)[i] = u32_to_float(mem[word_addr + i]);
                } else {
                    std::cout << "[BRAM] Warning: Read beyond BRAM bounds at word "
                              << (word_addr + i) << std::endl;
                }
            }
            pl.set_response_status(tlm::TLM_OK_RESPONSE);
        break;

        case tlm::TLM_WRITE_COMMAND:
            // Write 'len' words to BRAM; store float bit pattern as uint32_t
            for(uint32_t i = 0; i < len; i++){
                if (word_addr + i < BRAM_SIZE) {
                    mem[word_addr + i] = float_to_u32(reinterpret_cast<float*>(buf)[i]);
                } else {
                    std::cout << "[BRAM] Warning: Write beyond BRAM bounds at word "
                              << (word_addr + i) << std::endl;
                }
            }
            pl.set_response_status(tlm::TLM_OK_RESPONSE);
        break;

        default:
            std::cout << "[BRAM] Error: Unknown TLM command" << std::endl;
            pl.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
        break;
    }
    
    // BRAM access latency: 10ns (typical for FPGA BRAM)
    wait(15, sc_core::SC_NS);
}