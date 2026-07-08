/**
 * @file ahrs_hw.h
 * @brief AHRS Hardware Accelerator Interface - Sections 5-8
 * 
 * HW accelerator receives a BATCH of samples and processes them in a for loop.
 * This is a more realistic representation of HW implementation for VHDL.
 */

#ifndef AHRS_HW_H
#define AHRS_HW_H

#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//------------------------------------------------------------------------------
// Configuration
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
// HW Input Structure - Single Sample
//------------------------------------------------------------------------------

typedef struct {
    float gx, gy, gz;           // Gyroscope (deg/s)
    float ax, ay, az;           // Accelerometer (g)
    float halfGravityX;         // Half gravity vector (from SW Section 4)
    float halfGravityY;
    float halfGravityZ;
    float deltaTime;            // Time step (s)
    float rampedGain;           // Gain from SW Section 3
} HwInputSample;

//------------------------------------------------------------------------------
// HW Output Structure - Single Sample
//------------------------------------------------------------------------------

typedef struct {
    float qw, qx, qy, qz;       // Quaternion (UNNORMALIZED)
} HwOutputSample;

//------------------------------------------------------------------------------
// HW FUNCTION - Sections 5-8 with FOR LOOP
// 
// Processes a batch of numSamples samples (max HW_BATCH_SIZE)
// Has internal FOR loop - key for VHDL implementation!
//
// @param inputs      Array of input samples (prepared by SW)
// @param outputs     Array of output quaternions (unnormalized)
// @param numSamples  Number of samples in batch (1 to HW_BATCH_SIZE)
// HW FUNCTION - Sections 5-8 for a single sample
void AhrsHW_Sections5to8(
    FusionAhrs *const ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float halfGravityX, float halfGravityY, float halfGravityZ,
    float deltaTime,
    float *newQw, float *newQx, float *newQy, float *newQz
);

#endif // AHRS_HW_H