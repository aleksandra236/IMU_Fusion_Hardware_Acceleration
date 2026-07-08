/**
 * @file main_profile_realtime.cpp
 * @brief Real-time profiling version - simulates HW/SW batch processing
 * 
 * ARCHITECTURE:
 *   SW prepares batch of 10 samples (Sections 1-4)
 *   HW processes batch in FOR loop (Sections 5-8)
 *   SW normalizes results (Sections 9-11)
 * 
 * HW is called every 10 samples (100ms @ 100Hz)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "ahrs_hw.h"

#define SAMPLE_RATE 100
#define NUM_SAMPLES 2100

// Union for Fast Inverse Sqrt
typedef union {
    float f;
    int32_t i;
} Union32;

//------------------------------------------------------------------------------
// Initialization
//------------------------------------------------------------------------------

void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    memset(ahrs, 0, sizeof(FusionAhrs));
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;
    ahrs->initialising = true;
    ahrs->rampedGain = 10.0f;
    ahrs->gain = 0.5f;
    ahrs->gyroscopeRange = 2000.0f;
    ahrs->accelerationRejection = 10.0f;
    ahrs->recoveryTriggerPeriod = 500;
    ahrs->rampedGainStep = (10.0f - 0.5f) / 3.0f;
    ahrs->accelerationRecoveryTimeout = 500;
}

//------------------------------------------------------------------------------
// SW PHASE 1: Prepare single sample (Sections 1-4)
// This executes before sending to HW
//------------------------------------------------------------------------------

void SW_PrepareSample(
    FusionAhrs *const ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float deltaTime,
    HwInputSample *hwInput
) {
    // SECTION 1: STORE ACCELEROMETER
    ahrs->accelerometer.axis.x = ax;
    ahrs->accelerometer.axis.y = ay;
    ahrs->accelerometer.axis.z = az;
    
    // SECTION 2: GYROSCOPE RANGE CHECK
    if ((fabsf(gx) > ahrs->gyroscopeRange) || 
        (fabsf(gy) > ahrs->gyroscopeRange) || 
        (fabsf(gz) > ahrs->gyroscopeRange)) {
        ahrs->angularRateRecovery = true;
    }
    
    // SECTION 3: RAMP GAIN
    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if ((ahrs->rampedGain < ahrs->gain) || (ahrs->gain == 0.0f)) {
            ahrs->rampedGain = ahrs->gain;
            ahrs->initialising = false;
            ahrs->angularRateRecovery = false;
        }
    }
    
    // SECTION 4: HALF GRAVITY
    const float qw = ahrs->quaternion.element.w;
    const float qx = ahrs->quaternion.element.x;
    const float qy = ahrs->quaternion.element.y;
    const float qz = ahrs->quaternion.element.z;
    
    // Fill HW input structure
    hwInput->gx = gx;
    hwInput->gy = gy;
    hwInput->gz = gz;
    hwInput->ax = ax;
    hwInput->ay = ay;
    hwInput->az = az;
    hwInput->halfGravityX = qx * qz - qw * qy;
    hwInput->halfGravityY = qy * qz + qw * qx;
    hwInput->halfGravityZ = qw * qw - 0.5f + qz * qz;
    hwInput->deltaTime = deltaTime;
    hwInput->rampedGain = ahrs->rampedGain;
}

//------------------------------------------------------------------------------
// SW PHASE 2: Process HW output (Sections 9-11)
// This executes after HW processing
//------------------------------------------------------------------------------

void SW_ProcessOutput(
    FusionAhrs *const ahrs,
    const HwOutputSample *hwOutput
) {
    // SECTION 9: QUATERNION NORMALIZE
    const float qw = hwOutput->qw;
    const float qx = hwOutput->qx;
    const float qy = hwOutput->qy;
    const float qz = hwOutput->qz;
    
    const float quatMagSquared = qw*qw + qx*qx + qy*qy + qz*qz;
    
    Union32 uq = {.f = quatMagSquared};
    uq.i = 0x5F1F1412 - (uq.i >> 1);
    const float quatInvMag = uq.f * (1.69000231f - 0.714158168f * quatMagSquared * uq.f * uq.f);
    
    ahrs->quaternion.element.w = qw * quatInvMag;
    ahrs->quaternion.element.x = qx * quatInvMag;
    ahrs->quaternion.element.y = qy * quatInvMag;
    ahrs->quaternion.element.z = qz * quatInvMag;
    
    // SECTION 10: ZERO HEADING (only during initialization)
    if (ahrs->initialising) {
        const float yaw = atan2f(
            ahrs->quaternion.element.w * ahrs->quaternion.element.z + 
            ahrs->quaternion.element.x * ahrs->quaternion.element.y,
            0.5f - ahrs->quaternion.element.y * ahrs->quaternion.element.y - 
            ahrs->quaternion.element.z * ahrs->quaternion.element.z
        );
        
        const float halfYaw = 0.5f * yaw;
        const float cosHalfYaw = cosf(halfYaw);
        const float sinHalfYaw = sinf(halfYaw);
        
        const float rotW = cosHalfYaw;
        const float rotZ = -sinHalfYaw;
        
        const float rqw = ahrs->quaternion.element.w;
        const float rqx = ahrs->quaternion.element.x;
        const float rqy = ahrs->quaternion.element.y;
        const float rqz = ahrs->quaternion.element.z;
        
        ahrs->quaternion.element.w = rotW * rqw - rotZ * rqz;
        ahrs->quaternion.element.x = rotW * rqx - rotZ * rqy;
        ahrs->quaternion.element.y = rotW * rqy + rotZ * rqx;
        ahrs->quaternion.element.z = rotW * rqz + rotZ * rqw;
    }
    
    // SECTION 11: OUTPUT is implicit - quaternion is updated in ahrs
}

//------------------------------------------------------------------------------
// Real-time processing simulation with BATCH HW calls
//------------------------------------------------------------------------------

void simulate_realtime_processing(FusionAhrs* ahrs) {
    
    const float deltaTime = 0.01f;  // 100 Hz = 10ms
    
    HwInputSample hwInputBatch[HW_BATCH_SIZE];
    HwOutputSample hwOutputBatch[HW_BATCH_SIZE];
    
    int batchIndex = 0;
    int totalBatches = 0;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        
        // Simulated sensor data
        float gx = 2.0f + 0.01f * i;
        float gy = 0.5f + 0.005f * i;
        float gz = 0.1f;
        float ax = 0.01f;
        float ay = 0.02f;
        float az = 1.0f;
        
        //======================================================================
        // SW PHASE 1: Prepare sample for HW (Sections 1-4)
        //======================================================================
        SW_PrepareSample(ahrs, gx, gy, gz, ax, ay, az, deltaTime, &hwInputBatch[batchIndex]);
        
        batchIndex++;
        
        //======================================================================
        // When batch is full, call HW accelerator
        //======================================================================
        if (batchIndex >= HW_BATCH_SIZE) {
            
            //==================================================================
            // HW PHASE: Batch processing (Sections 5-8) - HAS FOR LOOP INSIDE!
            //==================================================================
            AhrsHW_ProcessBatch(hwInputBatch, hwOutputBatch, HW_BATCH_SIZE, ahrs);
            
            //==================================================================
            // SW PHASE 2: Process each HW output (Sections 9-11)
            //==================================================================
            for (int j = 0; j < HW_BATCH_SIZE; j++) {
                SW_ProcessOutput(ahrs, &hwOutputBatch[j]);
            }
            
            batchIndex = 0;
            totalBatches++;
        }
    }
    
    // Process remaining samples
    if (batchIndex > 0) {
        AhrsHW_ProcessBatch(hwInputBatch, hwOutputBatch, batchIndex, ahrs);
        for (int j = 0; j < batchIndex; j++) {
            SW_ProcessOutput(ahrs, &hwOutputBatch[j]);
        }
        totalBatches++;
    }
}

//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------

int main(void) {
    
    printf("HW_BATCH_SIZE = %d samples\n", HW_BATCH_SIZE);
    printf("Total samples: %d\n", NUM_SAMPLES);
    printf("Number of HW calls: %d\n\n", (NUM_SAMPLES + HW_BATCH_SIZE - 1) / HW_BATCH_SIZE);
    
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    
    simulate_realtime_processing(&ahrs);
    
    printf("Final: Q=[%.4f, %.4f, %.4f, %.4f]\n",
           ahrs.quaternion.element.w,
           ahrs.quaternion.element.x,
           ahrs.quaternion.element.y,
           ahrs.quaternion.element.z);
    
    return 0;
}