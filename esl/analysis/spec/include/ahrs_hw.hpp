/**
 * @file ahrs_hw.hpp
 * @brief AHRS Hardware Accelerator Interface - Sections 5-8 (fixed-point)
 *
 * Structs identični sa cpp_spec/spec/include/ahrs_hw.h (float referenca).
 * Implementacija u ahrs_hw.cpp koristi sc_fixed tipove.
 */

#ifndef AHRS_HW_HPP
#define AHRS_HW_HPP

#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//------------------------------------------------------------------------------
// Fusion types (float - isti interfejs kao float referenca)
//------------------------------------------------------------------------------

typedef union {
    float array[3];
    struct { float x, y, z; } axis;
} FusionVector;

typedef union {
    float array[4];
    struct { float w, x, y, z; } element;
} FusionQuaternion;

typedef struct {
    float array[3][3];
} FusionMatrix;

typedef struct {
    struct { float roll, pitch, yaw; } angle;
} FusionEuler;

#define FUSION_VECTOR_ZERO ((FusionVector){ .array = {0.0f, 0.0f, 0.0f} })
#define FUSION_VECTOR_ONES ((FusionVector){ .array = {1.0f, 1.0f, 1.0f} })
#define FUSION_IDENTITY_QUATERNION ((FusionQuaternion){ .array = {1.0f, 0.0f, 0.0f, 0.0f} })
#define FUSION_IDENTITY_MATRIX ((FusionMatrix){ .array = {{1,0,0},{0,1,0},{0,0,1}} })

//------------------------------------------------------------------------------
// AHRS State
//------------------------------------------------------------------------------

typedef struct {
    float filterCoefficient;
    unsigned int timeout;
    unsigned int timer;
    FusionVector gyroscopeOffset;
} FusionOffset;

typedef struct {
    float gain;
    float gyroscopeRange;
    float accelerationRejection;
    unsigned int recoveryTriggerPeriod;
    FusionQuaternion quaternion;
    FusionVector accelerometer;
    bool initialising;
    float rampedGain;
    float rampedGainStep;
    bool angularRateRecovery;
    FusionVector halfAccelerometerFeedback;
    bool accelerometerIgnored;
    int accelerationRecoveryTrigger;
    int accelerationRecoveryTimeout;
} FusionAhrs;

//------------------------------------------------------------------------------
// HW Input/Output structures (za eventualni batch interfejs)
//------------------------------------------------------------------------------

typedef struct {
    float gx, gy, gz;
    float ax, ay, az;
    float halfGravityX, halfGravityY, halfGravityZ;
    float deltaTime;
    float rampedGain;
} HwInputSample;

typedef struct {
    float qw, qx, qy, qz;
} HwOutputSample;

//------------------------------------------------------------------------------
// HW FUNCTION - Sections 5-8, jedan uzorak
//------------------------------------------------------------------------------
void AhrsHW_Sections5to8(
    FusionAhrs *const ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float halfGravityX, float halfGravityY, float halfGravityZ,
    float deltaTime,
    float *newQw, float *newQx, float *newQy, float *newQz
);

#endif // AHRS_HW_HPP
