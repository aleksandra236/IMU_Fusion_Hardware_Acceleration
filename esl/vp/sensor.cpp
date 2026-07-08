#include "sensor.hpp"
#include <cstdio>

Sensor::Sensor(sc_core::sc_module_name name,
               const char *csv_path,
               int         max_samples)
    : sc_core::sc_module(name), tsock("tsock"), sample_idx_(0)
{
    tsock.register_b_transport(this, &Sensor::b_transport);

    // ── Load CSV samples ─────────────────────────────────────────────────────
    FILE *f = std::fopen(csv_path, "r");
    if (!f) {
        std::cerr << "[SENSOR] ERROR: cannot open CSV: " << csv_path << std::endl;
        sc_core::sc_stop();
        return;
    }

    // Skip header line
    char line[512];
    std::fgets(line, sizeof(line), f);

    int loaded = 0;
    while (std::fgets(line, sizeof(line), f) && loaded < max_samples) {
        float t, gx, gy, gz, ax, ay, az;
        if (std::sscanf(line, "%f,%f,%f,%f,%f,%f,%f",
                        &t, &gx, &gy, &gz, &ax, &ay, &az) != 7)
            continue;
        RawSensorSample s;
        s.gx = gx;  s.gy = gy;  s.gz = gz;
        s.ax = ax;  s.ay = ay;  s.az = az;
        samples_.push_back(s);
        loaded++;
    }
    std::fclose(f);

    std::cout << "[SENSOR] Loaded " << loaded
              << " samples from " << csv_path << std::endl;
}

void Sensor::b_transport(tlm::tlm_generic_payload &pl, sc_core::sc_time &offset)
{
    (void)offset;

    if (pl.get_command() != tlm::TLM_READ_COMMAND) {
        pl.set_response_status(tlm::TLM_COMMAND_ERROR_RESPONSE);
        return;
    }

    // Pick the next sample; clamp to last entry if we run out
    if (sample_idx_ >= samples_.size())
        sample_idx_ = samples_.size() - 1;

    const RawSensorSample &s = samples_[sample_idx_++];

    float *out          = reinterpret_cast<float *>(pl.get_data_ptr());
    const float *words  = reinterpret_cast<const float *>(&s);
    int          len    = pl.get_data_length();
    for (int i = 0; i < len && i < RAW_SENSOR_WORDS; i++)
        out[i] = words[i];

    pl.set_response_status(tlm::TLM_OK_RESPONSE);
    wait(10, sc_core::SC_NS);  // 1 clock cycle @ 10 ns
}
