#include "ddr.hpp"
#include <cstdio>

DDR::DDR(sc_core::sc_module_name name)
    : sc_core::sc_module(name), tsock("tsock")
{
    tsock.register_b_transport(this, &DDR::b_transport);

    for (int i = 0; i < DDR_SIZE; i++) {
        mem[i] = 0;
    }

    std::cout << "[DDR] Initialized: "
              << DDR_SIZE << " words × 32-bit ("
              << (DDR_SIZE * 4 / 1024) << " KB)" << std::endl;
}

void DDR::b_transport(tlm::tlm_generic_payload &pl, sc_core::sc_time &offset)
{
    (void)offset;

    uint32_t         addr = (uint32_t)pl.get_address();
    uint32_t         len  = (uint32_t)pl.get_data_length();
    unsigned char   *buf  = pl.get_data_ptr();
    tlm::tlm_command cmd  = pl.get_command();

    // Bounds check
    if ((addr + len) > DDR_SIZE) {
        std::cout << "[DDR] Error: address out of range:"
                  << " addr=" << addr << " len=" << len
                  << " (max=" << DDR_SIZE << ")" << std::endl;
        pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        return;
    }

    switch (cmd) {
        case tlm::TLM_READ_COMMAND:
            for (uint32_t i = 0; i < len; i++) {
                reinterpret_cast<float *>(buf)[i] = u32_to_float(mem[addr + i]);
            }
            pl.set_response_status(tlm::TLM_OK_RESPONSE);
            break;

        case tlm::TLM_WRITE_COMMAND:
            for (uint32_t i = 0; i < len; i++) {
                mem[addr + i] = float_to_u32(reinterpret_cast<float *>(buf)[i]);
            }
            pl.set_response_status(tlm::TLM_OK_RESPONSE);
            break;

        default:
            std::cout << "[DDR] Error: unknown command." << std::endl;
            pl.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
            break;
    }

    // DDR access latency: 1 clock cycle @ 10 ns
    wait(10, sc_core::SC_NS);
}

// ============================================================================
// dump_csv – write normalized output quaternions to file after simulation
// ============================================================================
void DDR::dump_csv(const char *path, int n_samples) const
{
    std::FILE *f = std::fopen(path, "w");
    if (!f) {
        std::cerr << "[DDR] ERROR: cannot write to " << path << std::endl;
        return;
    }

    std::fprintf(f, "sample,qw,qx,qy,qz\n");
    for (int i = 0; i < n_samples; i++) {
        int base = i * OUTPUT_SAMPLE_WORDS;
        std::fprintf(f, "%d,%.8f,%.8f,%.8f,%.8f\n",
                     i,
                     u32_to_float(mem[base + 0]), u32_to_float(mem[base + 1]),
                     u32_to_float(mem[base + 2]), u32_to_float(mem[base + 3]));
    }

    std::fclose(f);
    std::cout << "[DDR] VP output written to " << path
              << "  (" << n_samples << " quaternions)" << std::endl;
}
