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
#define SAMPLE_RATE_HZ      100      // Sensor sample rate
#define SAMPLE_PERIOD_NS    10000000 // 10ms u nanosekundama

// ============================================================================
// Fixed-Point Configuration (from analysis)
// ============================================================================
#define QUAT_FRAC_BITS      27
#define NORM_FRAC_BITS      24
#define GYRO_FRAC_BITS      20
#define ACCEL_FRAC_BITS     20

// ============================================================================
// Memory Map
// ============================================================================
#define BRAM_SIZE           512
#define BRAM_ADDR_OFFSET    0x0000
#define BRAM_ADDR_MAX       0x01FF

// BRAM Memory Regions (word addresses)
#define BRAM_CTRL_OFFSET    0x0000  // Control registers (16 words)
#define BRAM_STATE_OFFSET   0x0010  // AHRS state (16 words)
#define BRAM_RAW_OFFSET     0x0020  // Raw sensor buffer SPI→BRAM (30 words)
// Sample-by-sample: uvek se koristi samo index 0 u input/output baferima
#define BRAM_INPUT_OFFSET   0x005C  // HW input buffer CPU→BRAM (jedan HwInputSample = 11 words)
#define BRAM_OUTPUT_OFFSET  0x00CA  // HW output buffer IP→BRAM (jedan HwOutputSample = 4 words)
#define BRAM_RESERVED       0x00F2

// SPI Controller
#define SPI_ADDR_OFFSET     0x0300
#define SPI_ADDR_MAX        0x001F

// AHRS IP Registers
#define IP_ADDR_OFFSET      0x0200
#define IP_ADDR_MAX         0x000F

#define IP_REG_CTRL         0x0000
#define IP_REG_STATUS       0x0001
#define IP_REG_SAMPLE_SIZE  0x0002
#define IP_REG_SAMPLE_CNT   0x0003
#define IP_REG_ERROR_CODE   0x0004

// ============================================================================
// Data Structure Sizes (in 32-bit words)
// ============================================================================
#define RAW_SENSOR_WORDS    6   // gx,gy,gz,ax,ay,az
#define INPUT_SAMPLE_WORDS  15  // gx,gy,gz,ax,ay,az,hgX,hgY,hgZ,dt,gain,qw,qx,qy,qz
#define OUTPUT_SAMPLE_WORDS 4   // qw,qx,qy,qz (nenormalizovan)
#define QUAT_STATE_WORDS    4

// ============================================================================
// Control Register Bits
// ============================================================================
#define CTRL_BIT_START      0
#define CTRL_BIT_RESET      1
#define CTRL_BIT_MODE       2

#define STATUS_BIT_BUSY     0
#define STATUS_BIT_DONE     1
#define STATUS_BIT_ERROR    2

// ============================================================================
// Global Events
// ============================================================================
extern sc_core::sc_event ahrs_done;
extern sc_core::sc_event sensor_ready;

// ============================================================================
// DDR Memory
// ============================================================================
#define DDR_SIZE            8192

// ============================================================================
// Simulation Control
// ============================================================================
#define MAX_SAMPLES         1918 // Ukupan broj uzoraka

// HLS sinteza: 65 ciklusa × 15ns clock = 975ns po uzorku
// (Vitis HLS 2025.1, xc7z010clg400-1, clock=15ns, ALLOCATION mul limit=26)
#define AHRS_IP_LATENCY_NS  975

// ============================================================================
// Data Structures
// ============================================================================
struct RawSensorSample {
    float gx, gy, gz;
    float ax, ay, az;
};  // 6 floats = RAW_SENSOR_WORDS

struct HwInputSample {
    float gx, gy, gz;
    float ax, ay, az;
    float halfGravityX;
    float halfGravityY;
    float halfGravityZ;
    float deltaTime;
    float rampedGain;
    float qw, qx, qy, qz;  // normalizovani kvaternion od CPU (SW9)
};  // 15 floats = INPUT_SAMPLE_WORDS

struct HwOutputSample {
    float qw, qx, qy, qz;
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

#define DDR_RING_SAMPLES  (DDR_SIZE / OUTPUT_SAMPLE_WORDS)

// ============================================================================
// Debug Flags
// ============================================================================
// #define DEBUG_INTERCONNECT
// #define DEBUG_BRAM
// #define DEBUG_IP
// #define DEBUG_CPU

#endif