#ifndef SPI_HPP
#define SPI_HPP

#include "defines.hpp"

/**
 * @brief SPI Controller
 *
 * Bridges the IMU Sensor and the system Interconnect.
 *
 * Processing loop (SC_THREAD):
 *   1. Wait one sample period (SAMPLE_PERIOD_NS).
 *   2. Read one RawSensorSample (6 floats) from Sensor via isock_sensor.
 *   3. Store the sample in an internal buffer.
 *   4. After HW_BATCH_SIZE samples → notify sensor_ready.
 *
 * The CPU reads the buffered raw data through the Interconnect
 * (Interconnect initiates READ transactions to tsock_interconnect).
 *
 * Socket layout:
 *   isock_sensor        → Sensor::tsock
 *   tsock_interconnect  ← Interconnect::isock_spi
 */
class SPI : public sc_core::sc_module {
public:
    SC_HAS_PROCESS(SPI);
    SPI(sc_core::sc_module_name name);

    tlm_utils::simple_initiator_socket<SPI> isock_sensor;
    tlm_utils::simple_target_socket<SPI>    tsock_interconnect;

    void b_transport(tlm::tlm_generic_payload&, sc_core::sc_time&);

private:
    void spi_process();

    int      batch_idx;      // current sample index within batch (0..HW_BATCH_SIZE-1)
    uint32_t total_batches;  // total batches sent

    // Internal buffer: stores raw sensor data until CPU reads it
    float raw_buffer[HW_BATCH_SIZE * RAW_SENSOR_WORDS];
};

#endif
