#ifndef DDR_HPP
#define DDR_HPP

#include "defines.hpp"

/**
 * @brief DDR Memory (CPU-private)
 *
 * General-purpose memory directly connected to the CPU (not routed through
 * the Interconnect).  The CPU uses it to store the AHRS output quaternions.
 *
 * Storage: DDR_SIZE floats = 4 KB
 *   Word 0 .. (MAX_SAMPLES × OUTPUT_SAMPLE_WORDS − 1)
 *   are used for quaternion results; the rest is available for future use.
 *
 * Socket layout:
 *   tsock  ← CPU::isock_ddr
 */
class DDR : public sc_core::sc_module {
public:
    DDR(sc_core::sc_module_name name);

    tlm_utils::simple_target_socket<DDR> tsock;

    void b_transport(tlm::tlm_generic_payload&, sc_core::sc_time&);

    /**
     * @brief Dump normalized output quaternions to a CSV file.
     * @param path       Output file path.
     * @param n_samples  Number of samples (each stored as 4 consecutive floats).
     */
    void dump_csv(const char *path, int n_samples) const;

private:
    // DDR storage: DDR_SIZE words × 32-bit (IEEE 754 float bit patterns)
    uint32_t mem[DDR_SIZE];
};

#endif
