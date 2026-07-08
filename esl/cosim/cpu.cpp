#include "cpu.hpp"

CPU::CPU(sc_core::sc_module_name name)
    : sc_core::sc_module(name),
      isock_interconnect("isock_interconnect"),
      isock_ddr("isock_ddr")
{
    SC_THREAD(cpu_process);

    qw = 1.0f;  qx = 0.0f;  qy = 0.0f;  qz = 0.0f;

    gain             = 0.5f;
    gyroscopeRange   = 2000.0f;
    rampedGain       = 10.0f;
    rampedGainStep   = (10.0f - gain) / 3.0f;

    sample_count        = 0;
    initialising        = true;
    angularRateRecovery = false;

    std::cout << "[CPU] Initialized. Sample-by-sample rezim. "
              << MAX_SAMPLES << " uzoraka ukupno." << std::endl;
}

// ============================================================================
// Glavni procesni thread – sample-by-sample
// ============================================================================
void CPU::cpu_process()
{
    const float deltaTime = 1.0f / SAMPLE_RATE_HZ;

    while (true) {

        // 1. Cekaj jedan sirovi uzorak od SPI
        wait(sensor_ready);
        sample_count++;

        std::cout << "\n[CPU] *** Sample " << sample_count
                  << "/" << MAX_SAMPLES << " ***" << std::endl;

        // 2. Citaj sirovi uzorak iz SPI bafera
        RawSensorSample raw;
        raw.gx = read_interconnect(SPI_ADDR_OFFSET + 0);
        raw.gy = read_interconnect(SPI_ADDR_OFFSET + 1);
        raw.gz = read_interconnect(SPI_ADDR_OFFSET + 2);
        raw.ax = read_interconnect(SPI_ADDR_OFFSET + 3);
        raw.ay = read_interconnect(SPI_ADDR_OFFSET + 4);
        raw.az = read_interconnect(SPI_ADDR_OFFSET + 5);

        // SW Sekcije 1-4: pripremi HwInputSample
        HwInputSample hwIn;
        sw_prepare_sample(raw, deltaTime, hwIn);

        // Upisi HwInputSample u BRAM
        write_interconnect(BRAM_INPUT_OFFSET + 0,  hwIn.gx);
        write_interconnect(BRAM_INPUT_OFFSET + 1,  hwIn.gy);
        write_interconnect(BRAM_INPUT_OFFSET + 2,  hwIn.gz);
        write_interconnect(BRAM_INPUT_OFFSET + 3,  hwIn.ax);
        write_interconnect(BRAM_INPUT_OFFSET + 4,  hwIn.ay);
        write_interconnect(BRAM_INPUT_OFFSET + 5,  hwIn.az);
        write_interconnect(BRAM_INPUT_OFFSET + 6,  hwIn.halfGravityX);
        write_interconnect(BRAM_INPUT_OFFSET + 7,  hwIn.halfGravityY);
        write_interconnect(BRAM_INPUT_OFFSET + 8,  hwIn.halfGravityZ);
        write_interconnect(BRAM_INPUT_OFFSET + 9,  hwIn.deltaTime);
        write_interconnect(BRAM_INPUT_OFFSET + 10, hwIn.rampedGain);
        write_interconnect(BRAM_INPUT_OFFSET + 11, hwIn.qw);
        write_interconnect(BRAM_INPUT_OFFSET + 12, hwIn.qx);
        write_interconnect(BRAM_INPUT_OFFSET + 13, hwIn.qy);
        write_interconnect(BRAM_INPUT_OFFSET + 14, hwIn.qz);

        // Posalji START AHRS IP-u
        write_interconnect(IP_ADDR_OFFSET + IP_REG_CTRL, 1.0f);

        // Cekaj interrupt od AHRS IP-a
        wait(ahrs_done);

        // Citaj rezultat iz BRAM-a
        HwOutputSample hwOut;
        hwOut.qw = read_interconnect(BRAM_OUTPUT_OFFSET + 0);
        hwOut.qx = read_interconnect(BRAM_OUTPUT_OFFSET + 1);
        hwOut.qy = read_interconnect(BRAM_OUTPUT_OFFSET + 2);
        hwOut.qz = read_interconnect(BRAM_OUTPUT_OFFSET + 3);

        // SW Sekcije 9-11: normalizuj kvaternion
        sw_process_output(hwOut);

        // Upisi normalizovani kvaternion u DDR
        uint32_t ddr_addr = ((sample_count - 1) % DDR_RING_SAMPLES) * OUTPUT_SAMPLE_WORDS;
        write_ddr(ddr_addr + 0, qw);
        write_ddr(ddr_addr + 1, qx);
        write_ddr(ddr_addr + 2, qy);
        write_ddr(ddr_addr + 3, qz);

        std::cout << "[CPU] Sample " << sample_count
                  << " done. q=("
                  << qw << ", " << qx << ", " << qy << ", " << qz << ")" << std::endl;

        if (sample_count >= MAX_SAMPLES) {
            std::cout << "\n[CPU] Svih " << MAX_SAMPLES
                      << " uzoraka obradeno. Zaustavljam simulaciju." << std::endl;
            sc_core::sc_stop();
            return;
        }
    }
}

// ============================================================================
// SW Phase 1 – Sekcije 1-4
// ============================================================================
void CPU::sw_prepare_sample(const RawSensorSample &raw,
                            float deltaTime,
                            HwInputSample &out)
{
    // SEKCIJA 1: Akcelerometar
    out.ax = raw.ax;
    out.ay = raw.ay;
    out.az = raw.az;

    // SEKCIJA 2: Provera opsega ziroskopa
    if (fabsf(raw.gx) > gyroscopeRange ||
        fabsf(raw.gy) > gyroscopeRange ||
        fabsf(raw.gz) > gyroscopeRange) {
        angularRateRecovery = true;
    }
    out.gx = raw.gx;
    out.gy = raw.gy;
    out.gz = raw.gz;

    // SEKCIJA 3: Pojacanje rampa tokom inicijalizacije
    if (initialising) {
        rampedGain -= rampedGainStep * deltaTime;
        if (rampedGain < gain || gain == 0.0f) {
            rampedGain   = gain;
            initialising = false;
        }
    }
    out.rampedGain = rampedGain;

    // SEKCIJA 4: halfGravity iz trenutnog kvaterniona
    out.halfGravityX = qx * qz - qw * qy;
    out.halfGravityY = qy * qz + qw * qx;
    out.halfGravityZ = qw * qw - 0.5f + qz * qz;

    out.deltaTime = deltaTime;

    // SEKCIJA 4 (dopuna): proslijedi normalizovani q AHRS_IP-u za integraciju
    out.qw = qw;  out.qx = qx;  out.qy = qy;  out.qz = qz;
}

// ============================================================================
// SW Phase 2 – Sekcije 9-11
// ============================================================================
void CPU::sw_process_output(const HwOutputSample &hwOut)
{
    // SEKCIJA 9: Normalizacija (fast inverse sqrt)
    float nqw = hwOut.qw, nqx = hwOut.qx, nqy = hwOut.qy, nqz = hwOut.qz;

    float magSq = nqw*nqw + nqx*nqx + nqy*nqy + nqz*nqz;

    union { float f; int32_t i; } u;
    u.f = magSq;
    u.i = 0x5F1F1412 - (u.i >> 1);
    float invMag = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);

    qw = nqw * invMag;
    qx = nqx * invMag;
    qy = nqy * invMag;
    qz = nqz * invMag;

    // SEKCIJA 10: Zero-heading korekcija (samo tokom inicijalizacije)
    if (initialising) {
        float yaw = atan2f(qw * qz + qx * qy,
                           0.5f - qy * qy - qz * qz);
        float halfYaw    = 0.5f * yaw;
        float cosHalfYaw = cosf(halfYaw);
        float sinHalfYaw = sinf(halfYaw);

        const float newQw = cosHalfYaw * qw + sinHalfYaw * qz;
        const float newQx = cosHalfYaw * qx + sinHalfYaw * qy;
        const float newQy = cosHalfYaw * qy - sinHalfYaw * qx;
        const float newQz = cosHalfYaw * qz - sinHalfYaw * qw;

        qw = newQw;  qx = newQx;  qy = newQy;  qz = newQz;
    }
}

// ============================================================================
// TLM helpers
// ============================================================================
void CPU::write_interconnect(int addr, float value)
{
    tlm::tlm_generic_payload pl;
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME;
    float                    data  = value;

    pl.set_command(tlm::TLM_WRITE_COMMAND);
    pl.set_address(addr);
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data));
    pl.set_data_length(1);
    pl.set_streaming_width(1);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_interconnect->b_transport(pl, delay);
}

float CPU::read_interconnect(int addr)
{
    tlm::tlm_generic_payload pl;
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME;
    float                    data  = 0.0f;

    pl.set_command(tlm::TLM_READ_COMMAND);
    pl.set_address(addr);
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data));
    pl.set_data_length(1);
    pl.set_streaming_width(1);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_interconnect->b_transport(pl, delay);
    return data;
}

void CPU::write_ddr(int addr, float value)
{
    tlm::tlm_generic_payload pl;
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME;
    float                    data  = value;

    pl.set_command(tlm::TLM_WRITE_COMMAND);
    pl.set_address((uint64_t)addr);
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data));
    pl.set_data_length(1);
    pl.set_streaming_width(1);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_ddr->b_transport(pl, delay);
}