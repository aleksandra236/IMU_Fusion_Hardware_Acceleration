#include "spi.hpp"
#include <cstring>

SPI::SPI(sc_core::sc_module_name name)
    : sc_core::sc_module(name),
      isock_sensor("isock_sensor"),
      tsock_interconnect("tsock_interconnect")
{
    tsock_interconnect.register_b_transport(this, &SPI::b_transport);
    SC_THREAD(spi_process);

    total_samples = 0;
    std::memset(raw_buffer, 0, sizeof(raw_buffer));

    std::cout << "[SPI] Initialized."
              << " Sample rate: " << SAMPLE_RATE_HZ << " Hz"
              << ", Period: " << (SAMPLE_PERIOD_NS / 1000000) << " ms" << std::endl;
}

// ============================================================================
// b_transport – handle READ requests from CPU via Interconnect
// Address is a word offset into the raw_buffer (0 .. RAW_SENSOR_WORDS-1)
// ============================================================================
void SPI::b_transport(tlm::tlm_generic_payload &pl, sc_core::sc_time &offset)
{
    (void)offset;

    uint32_t         addr = (uint32_t)pl.get_address();
    unsigned char   *buf  = pl.get_data_ptr();
    tlm::tlm_command cmd  = pl.get_command();

    if (cmd == tlm::TLM_READ_COMMAND) {
        if (addr < (uint32_t)RAW_SENSOR_WORDS) {
            *reinterpret_cast<float *>(buf) = raw_buffer[addr];
            pl.set_response_status(tlm::TLM_OK_RESPONSE);
        } else {
            std::cout << "[SPI] Error: read address out of range: " << addr << std::endl;
            pl.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
        }
    } else {
        pl.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
    }

    wait(10, sc_core::SC_NS);  // 1 clock cycle @ 10 ns
}

// ============================================================================
// spi_process – SC_THREAD: reads from sensor, buffers internally
// ============================================================================
void SPI::spi_process()
{
    tlm::tlm_generic_payload pl;
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME;
    float                    raw_buf[RAW_SENSOR_WORDS];   // gx,gy,gz,ax,ay,az

    while (true) {

        // ── Step 1: Wait one sample period ──────────────────────────────────
        wait(SAMPLE_PERIOD_NS, sc_core::SC_NS);

        // ── Step 2: Read one raw sample (6 floats) from Sensor ───────────────
        pl.set_command(tlm::TLM_READ_COMMAND);
        pl.set_address(0);
        pl.set_data_ptr(reinterpret_cast<unsigned char *>(raw_buf));
        pl.set_data_length(RAW_SENSOR_WORDS);
        pl.set_streaming_width(RAW_SENSOR_WORDS);
        pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

        isock_sensor->b_transport(pl, delay);

        if (pl.get_response_status() != tlm::TLM_OK_RESPONSE) {
            std::cout << "[SPI] Error: sensor read failed at sample=" << total_samples << std::endl;
            continue;
        }

        // ── Step 3: Store raw sample in internal buffer ──────────────────────
        for (int w = 0; w < RAW_SENSOR_WORDS; w++) {
            raw_buffer[w] = raw_buf[w];
        }

        total_samples++;

        std::cout << "[SPI] Sample " << total_samples
                  << " ready. Notifying CPU..." << std::endl;

        sensor_ready.notify();
    }
}
