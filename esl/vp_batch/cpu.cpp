#include "cpu.hpp"

CPU::CPU(sc_core::sc_module_name name) // constructor definition
    : sc_core::sc_module(name), // initialize base sc_module with the given name
      isock_interconnect("isock_interconnect"), // initialize initiator socket for Interconnect communication
      isock_ddr("isock_ddr") // initialize initiator socket for DDR communication
{
    SC_THREAD(cpu_process); // register cpu_process as a SystemC thread process, 
    //it will run concurrently in the simulation

    // ── Initialise SW AHRS state (matches FusionAhrsInitialise) ─────────────
    // sc_fixed members accept float literals via implicit quantisation
    qw = 1.0f;  qx = 0.0f;  qy = 0.0f;  qz = 0.0f;  // Identity quaternion

    gain             = 0.5f;
    gyroscopeRange   = 2000.0f;     // deg/s
    rampedGain       = 10.0f;
    rampedGainStep   = (10.0f - gain) / 3.0f;

    batch_count         = 0;
    initialising        = true;
    angularRateRecovery = false;

    std::cout << "[CPU] Initialized. Will process up to "
              << MAX_BATCHES << " batches." << std::endl;
}

// ============================================================================
// Main processing thread
// ============================================================================
void CPU::cpu_process()
{
    const float deltaTime = 1.0f / SAMPLE_RATE_HZ;  // 0.01 s, time between samples

    while (true) { // runs forver until sc_stop() is called, which will happen after MAX_BATCHES

        // ── 1. Wait for a full batch of raw sensor data from SPI ────────────
        wait(sensor_ready); //blocks the CPU process until the sensor_ready event is notified, 
        //which indicates that a new batch of raw sensor data is available in the BRAM at BRAM_RAW_OFFSET. 
        //This synchronizes the CPU with the data production from the SPI interface, 
        //ensuring that it processes fresh data for each batch.
        batch_count++;

        std::cout << "\n[CPU] *** Batch " << batch_count
                  << "/" << MAX_BATCHES << " ─ SW Phase 1 (Sections 1-4) ***" << std::endl;

        // ── 2. SW Phase 1: for each raw sample → prepare HwInputSample ──────
        for (int i = 0; i < HW_BATCH_SIZE; i++) {

            // Read raw sensor sample from SPI controller (via Interconnect)
            int raw_addr = SPI_ADDR_OFFSET + i * RAW_SENSOR_WORDS; //calculate the base address for the i-th 
            //raw sensor sample in SPI buffer, where RAW_SENSOR_WORDS is the number of 32-bit words that 
            //represent one raw sensor sample (6 floats = 6 words).
            RawSensorSample raw;
            raw.gx = read_interconnect(raw_addr + 0);
            raw.gy = read_interconnect(raw_addr + 1);
            raw.gz = read_interconnect(raw_addr + 2);
            raw.ax = read_interconnect(raw_addr + 3);
            raw.ay = read_interconnect(raw_addr + 4);
            raw.az = read_interconnect(raw_addr + 5);

            // Prepare HwInputSample (SW Sections 1-4)
            HwInputSample hwIn;
            sw_prepare_sample(raw, deltaTime, hwIn);

            // Write prepared HwInputSample to BRAM_INPUT_OFFSET (field by field)
            uint32_t in_addr = BRAM_INPUT_OFFSET + i * INPUT_SAMPLE_WORDS;
            write_interconnect(in_addr + 0,  hwIn.gx);
            write_interconnect(in_addr + 1,  hwIn.gy);
            write_interconnect(in_addr + 2,  hwIn.gz);
            write_interconnect(in_addr + 3,  hwIn.ax);
            write_interconnect(in_addr + 4,  hwIn.ay);
            write_interconnect(in_addr + 5,  hwIn.az);
            write_interconnect(in_addr + 6,  hwIn.halfGravityX);
            write_interconnect(in_addr + 7,  hwIn.halfGravityY);
            write_interconnect(in_addr + 8,  hwIn.halfGravityZ);
            write_interconnect(in_addr + 9,  hwIn.deltaTime);
            write_interconnect(in_addr + 10, hwIn.rampedGain);
        }

        // ── 3. Send START to AHRS IP ─────────────────────────────────────────
        std::cout << "[CPU] Starting AHRS IP..." << std::endl;
        write_interconnect(IP_ADDR_OFFSET + IP_REG_CTRL, 1.0f);

        // ── 4. Wait for AHRS IP interrupt ────────────────────────────────────
        wait(ahrs_done);

        std::cout << "[CPU] Interrupt received. SW Phase 2 (Sections 9-11)..." << std::endl;

        // ── 5. SW Phase 2: normalise each HW output quaternion ───────────────
        // Ring-buffer write: wrap after DDR_RING_BATCHES so the simulation can
        // run for any number of batches without overflowing DDR.
        int ddr_base = ((batch_count - 1) % DDR_RING_BATCHES) * HW_BATCH_SIZE * OUTPUT_SAMPLE_WORDS;

        for (int i = 0; i < HW_BATCH_SIZE; i++) {
            uint32_t out_addr = BRAM_OUTPUT_OFFSET + i * OUTPUT_SAMPLE_WORDS;

            HwOutputSample hwOut;
            hwOut.qw = read_interconnect(out_addr + 0);
            hwOut.qx = read_interconnect(out_addr + 1);
            hwOut.qy = read_interconnect(out_addr + 2);
            hwOut.qz = read_interconnect(out_addr + 3);

            // SW Section 9-11: normalise, zero-heading
            sw_process_output(hwOut);

            // ── 6. Write normalised quaternion (float) to DDR ───────────────
            uint32_t ddr_addr = ddr_base + (uint32_t)(i * OUTPUT_SAMPLE_WORDS);
            write_ddr(ddr_addr + 0, qw);
            write_ddr(ddr_addr + 1, qx);
            write_ddr(ddr_addr + 2, qy);
            write_ddr(ddr_addr + 3, qz);
        }

        std::cout << "[CPU] Batch " << batch_count
                  << " done. Final q=("
                  << qw << ", " << qx << ", " << qy << ", " << qz << ")" << std::endl;

        // ── 7. Stop after MAX_BATCHES ─────────────────────────────────────────
        if (batch_count >= MAX_BATCHES) {
            std::cout << "\n[CPU] All " << MAX_BATCHES
                      << " batches processed. Stopping simulation." << std::endl;
            sc_core::sc_stop();
            return;
        }
    }
}

// ============================================================================
// SW Phase 1 – Sections 1-4
// Mirrors SW_PrepareSample() from main_profile_realtime.cpp
// ============================================================================
void CPU::sw_prepare_sample(const RawSensorSample &raw,
                            float deltaTime,
                            HwInputSample &out)
{
    // SECTION 1: Store accelerometer (implicit – pass through to out)
    // float → accel_t (sc_fixed): kvantizuje se na granici SW/HW
    out.ax = raw.ax;
    out.ay = raw.ay;
    out.az = raw.az;

    // SECTION 2: Gyroscope range check
    if (fabsf(raw.gx) > gyroscopeRange || //fabs - absolute value for float, checks if any angular rate exceeds the configured range, which would trigger angular rate recovery mode
        fabsf(raw.gy) > gyroscopeRange ||
        fabsf(raw.gz) > gyroscopeRange) {
        angularRateRecovery = true;
    }
    out.gx = raw.gx;
    out.gy = raw.gy;
    out.gz = raw.gz;

    // SECTION 3: Ramp gain during initialisation (SW: float aritmetika)
    if (initialising) {
        rampedGain -= rampedGainStep * deltaTime;
        if (rampedGain < gain || gain == 0.0f) {
            rampedGain   = gain;
            initialising = false;
        }
    }
    out.rampedGain = rampedGain;  // float → gain_t (kvantizuje se za HW)

    // SECTION 4: Compute halfGravity from current (float) quaternion.
    // SW koristi float direktno – nema potrebe za kastovanjem.
    out.halfGravityX = qx * qz - qw * qy;          // float → hg_t (za HW)
    out.halfGravityY = qy * qz + qw * qx;
    out.halfGravityZ = qw * qw - 0.5f + qz * qz;

    out.deltaTime = deltaTime;  // float → dt_t (kvantizuje se za HW)
}

// ============================================================================
// SW Phase 2 – Sections 9-11
// Mirrors SW_ProcessOutput() from main_profile_realtime.cpp
// ============================================================================
void CPU::sw_process_output(const HwOutputSample &hwOut) // takes the unnormalised quaternion output from 
//the AHRS IP, normalises it, applies zero-heading correction during initialisation, 
//and updates the CPU's internal quaternion state
{
    // SECTION 9: Fast inverse square root normalisation (Quake-style)
    float nqw = hwOut.qw, nqx = hwOut.qx, nqy = hwOut.qy, nqz = hwOut.qz; // copy unnormalised quaternion components to local variables for processing

    float magSq = nqw*nqw + nqx*nqx + nqy*nqy + nqz*nqz; // compute the squared magnitude of the quaternion, which is used for normalisation.

    // Union trick for fast inverse sqrt (matches spec exactly)
    union { float f; int32_t i; } u;
    u.f = magSq;
    u.i = 0x5F1F1412 - (u.i >> 1);
    float invMag = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f); // compute the inverse 
    //magnitude using the fast inverse square root method, which gives an approximation o
    //f 1/sqrt(magSq). This is used to normalise the quaternion.

    qw = nqw * invMag; // normalise the quaternion components by multiplying with the inverse magnitude
    qx = nqx * invMag;
    qy = nqy * invMag;
    qz = nqz * invMag;

    // SECTION 10: Zero-heading correction (only during initialisation)
    if (initialising) { //only applied during the initialisation phase, 
        //this correction rotates the quaternion to set the initial yaw (heading) to zero, 
        //while preserving the initial pitch and roll. 
        //This is done by calculating the current yaw from the quaternion, 
        //computing a rotation that would negate this yaw, and applying it to the quaternion. 
        //This ensures that the AHRS starts with a known heading reference, 
        //which can be important for applications like navigation or orientation tracking.
        // Yaw from quaternion: atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
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

    // SECTION 11: quaternion je sada ažuriran u CPU float state
}

// ── TLM Helpers - handle communication with Interconnect and DDR using TLM transactions ─────────────

void CPU::write_interconnect(int addr, float value)
{
    tlm::tlm_generic_payload pl; // create a TLM generic payload for the transaction
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME; // no delay for this simple model, but could be modified to simulate timing
    float                    data  = value; // data to be written, placed in a float variable to match the expected data type

    pl.set_command(tlm::TLM_WRITE_COMMAND);
    pl.set_address(addr);
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data)); // set data pointer to the address of the data variable, cast to unsigned char* as required by TLM
    pl.set_data_length(1);
    pl.set_streaming_width(1); // for burst transactions, but here we just write one word
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_interconnect->b_transport(pl, delay); // send the transaction through the initiator socket to the Interconnect
    // blocking transport call - the Interconnect will process the transaction and update the response status in the payload
}

float CPU::read_interconnect(int addr)
{
    tlm::tlm_generic_payload pl; // create a TLM generic payload for the transaction
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME;
    float                    data  = 0.0f; // variable to hold the data read from the Interconnect, initialized to 0

    pl.set_command(tlm::TLM_READ_COMMAND);
    pl.set_address(addr);
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data)); // set data pointer to the address of the data variable, cast to unsigned char* as required by TLM
    pl.set_data_length(1);
    pl.set_streaming_width(1);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_interconnect->b_transport(pl, delay); // send the transaction through the initiator socket to the Interconnect
    // blocking transport call - the Interconnect will process the transaction and update the response status in the payload

    return data; // this function returns the float value read from the Interconnect, which will have been updated by the b_transport call
}

void CPU::write_ddr(int addr, float value)
{
    tlm::tlm_generic_payload pl; // create a TLM generic payload for the transaction
    sc_core::sc_time         delay = sc_core::SC_ZERO_TIME; // no delay for this simple model, but could be modified to simulate timing
    float                    data  = value; // data to be written, placed in a float variable to match the expected data type

    pl.set_command(tlm::TLM_WRITE_COMMAND);
    pl.set_address((uint64_t)addr);
    pl.set_data_ptr(reinterpret_cast<unsigned char *>(&data));
    pl.set_data_length(1);
    pl.set_streaming_width(1);
    pl.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    isock_ddr->b_transport(pl, delay);
}

