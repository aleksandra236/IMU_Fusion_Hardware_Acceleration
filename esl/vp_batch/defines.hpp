#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <iostream>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

using namespace sc_core;
using namespace sc_dt;
using namespace tlm;
using namespace std;

// ============================================================================
// AHRS Configuration
// ============================================================================
#define HW_BATCH_SIZE       5       // Samples per batch
#define SAMPLE_RATE_HZ      100     // Sensor sample rate
#define SAMPLE_PERIOD_NS    10000000 // 10ms in nanoseconds (for SC_TIME)

// ============================================================================
// Fixed-Point Configuration (from analysis)
// ============================================================================
#define QUAT_FRAC_BITS      27      // Quaternion precision
#define NORM_FRAC_BITS      24      // Normalized values
#define GYRO_FRAC_BITS      20      // Gyroscope data
#define ACCEL_FRAC_BITS     20      // Accelerometer data

// ============================================================================
// Memory Map
// ============================================================================
// BRAM: 512 words × 32-bit = 2KB
#define BRAM_SIZE           512
#define BRAM_ADDR_OFFSET    0x0000
#define BRAM_ADDR_MAX       0x01FF

// BRAM Memory Regions (word addresses)
#define BRAM_CTRL_OFFSET    0x0000  // Control registers (16 words)
#define BRAM_STATE_OFFSET   0x0010  // AHRS state (quaternion, etc.) (16 words)
#define BRAM_RAW_OFFSET     0x0020  // Raw sensor buffer  SPI→BRAM  (gx,gy,gz,ax,ay,az)×5  = 30 words
#define BRAM_INPUT_OFFSET   0x005C  // HW input  buffer  CPU→BRAM  HwInputSample[5]  = 55 words
#define BRAM_OUTPUT_OFFSET  0x00CA  // HW output buffer  IP→BRAM   HwOutputSample[5]  = 20 words
#define BRAM_RESERVED       0x00F2  // Reserved/Debug

// SPI Controller (word addresses, accessed via Interconnect)
#define SPI_ADDR_OFFSET     0x0300
#define SPI_ADDR_MAX        0x001F  // 32 words (enough for 5×6=30 raw sensor words)

// AHRS IP Registers (word addresses)
#define IP_ADDR_OFFSET      0x0200
#define IP_ADDR_MAX         0x000F  // 16 registers

// Specific IP register offsets (relative to IP_ADDR_OFFSET)
#define IP_REG_CTRL         0x0000  // Control: [0]=start, [1]=reset, [2]=mode
#define IP_REG_STATUS       0x0001  // Status: [0]=busy, [1]=done, [2]=error
#define IP_REG_BATCH_SIZE   0x0002  // Number of samples in current batch
#define IP_REG_SAMPLE_CNT   0x0003  // Sample counter (for debugging)
#define IP_REG_ERROR_CODE   0x0004  // Error code if status[2]=1

// ============================================================================
// Data Structure Sizes (in 32-bit words)
// ============================================================================
// Raw sensor sample from SPI: gx,gy,gz, ax,ay,az = 6 floats
#define RAW_SENSOR_WORDS    6

// HwInputSample: gx,gy,gz, ax,ay,az, halfGravityX/Y/Z, deltaTime, rampedGain = 11 floats
#define INPUT_SAMPLE_WORDS  11

// HwOutputSample: qw,qx,qy,qz = 4 floats (UNNORMALIZED – SW normalizes in Sec.9)
#define OUTPUT_SAMPLE_WORDS 4

// Current quaternion state
#define QUAT_STATE_WORDS    4       

// ============================================================================
// Control Register Bits
// ============================================================================
#define CTRL_BIT_START      0       // Write 1 to start processing
#define CTRL_BIT_RESET      1       // Write 1 to reset IP
#define CTRL_BIT_MODE       2       // 0=batch, 1=continuous

#define STATUS_BIT_BUSY     0       // 1=processing, 0=idle
#define STATUS_BIT_DONE     1       // 1=batch complete
#define STATUS_BIT_ERROR    2       // 1=error occurred

// ============================================================================
// Global Events
// ============================================================================
extern sc_core::sc_event ahrs_done;      // AHRS batch complete
extern sc_core::sc_event sensor_ready;   // New sensor data available

// ============================================================================
// DDR Memory (CPU private, not routed through Interconnect)
// ============================================================================
#define DDR_SIZE            1024    // 1024 floats = 4 KB

// ============================================================================
// Simulation Control
// ============================================================================
#define MAX_BATCHES         40      // 200 samples / HW_BATCH_SIZE(5) = 40 batches


// HLS C Synthesis result: 1350 cycles @ 10ns clock (Zybo Z7-10)
#define AHRS_IP_LATENCY_NS  13500

// ============================================================================
// Data Structures
// ============================================================================
// Raw sensor data (written by SPI to BRAM_RAW_OFFSET)
struct RawSensorSample {
    float gx, gy, gz;   // Gyroscope     (deg/s)
    float ax, ay, az;   // Accelerometer (g)
};  // 6 floats = RAW_SENSOR_WORDS

// HW input prepared by CPU SW (Sections 1-4), written to BRAM_INPUT_OFFSET
struct HwInputSample {
    float gx, gy, gz;               // Gyroscope          (deg/s)
    float ax, ay, az;               // Accelerometer      (g)
    float halfGravityX;             // Half-gravity X (from SW Sec.4, derived from quaternion)
    float halfGravityY;             // Half-gravity Y
    float halfGravityZ;             // Half-gravity Z
    float deltaTime;                // Time step          (s)
    float rampedGain;               // Ramped gain from SW Sec.3
};  // 11 floats = INPUT_SAMPLE_WORDS

// HW output (UNNORMALIZED quaternion) – CPU normalizes in SW Sec.9
struct HwOutputSample {
    float qw, qx, qy, qz;          // Unnormalized quaternion
};  // 4 floats = OUTPUT_SAMPLE_WORDS

// ============================================================================
// Float <-> uint32_t bit-exact conversion helpers
// ============================================================================
#include <cstring>
static inline float u32_to_float(uint32_t v) {
    float f; std::memcpy(&f, &v, 4); return f;
}
static inline uint32_t float_to_u32(float f) {
    uint32_t v; std::memcpy(&v, &f, 4); return v;
}

// Ring-buffer depth for DDR output storage
#define DDR_RING_BATCHES  (DDR_SIZE / (HW_BATCH_SIZE * OUTPUT_SAMPLE_WORDS))

// ============================================================================
// Debug Flags (comment out to disable)
// ============================================================================
// #define DEBUG_INTERCONNECT    // Print interconnect transactions
// #define DEBUG_BRAM            // Print BRAM accesses
// #define DEBUG_IP              // Print IP operations
// #define DEBUG_CPU             // Print CPU operations

#endif