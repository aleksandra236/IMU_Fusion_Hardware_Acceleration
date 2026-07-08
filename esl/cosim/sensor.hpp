#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "defines.hpp"
#include <vector>

/**
 * @brief Real-data IMU Sensor – serves samples loaded from a CSV file
 *
 * Loads up to max_samples rows from a CSV file (columns: time,gx,gy,gz,ax,ay,az)
 * on construction.  On each READ request, the SPI controller receives the next
 * RawSensorSample in sequence.  When all samples have been delivered the last
 * row is repeated so the simulation does not stall.
 *
 * Socket layout:
 *   tsock  ← SPI Controller (READ requests)
 */
class Sensor : public sc_core::sc_module {
public:
    Sensor(sc_core::sc_module_name name,
           const char *csv_path,
           int         max_samples = 200);

    tlm_utils::simple_target_socket<Sensor> tsock;

    void b_transport(tlm::tlm_generic_payload&, sc_core::sc_time&);

    int loaded_count() const { return static_cast<int>(samples_.size()); }

private:
    std::vector<RawSensorSample> samples_;
    size_t                       sample_idx_;
};

#endif
