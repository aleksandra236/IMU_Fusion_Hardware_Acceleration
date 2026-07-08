/**
 * @file ahrs_hw.c
 * @brief AHRS Hardware Accelerator - Sections 5-8 with FOR LOOP + DEBUG PRINTS
 * 
 * This file simulates the HW accelerator that processes a BATCH of samples.
 * Key characteristic: FOR LOOP inside the HW module.
 * 
 * MODIFICATION: Added printf debug statements - NO OTHER CHANGES!
 * 
 * Sections executed in HW:
 *   5: Accelerometer Feedback  (2x InvSqrt)
 *   6: Gyroscope Conversion
 *   7: Apply Feedback
 *   8: Quaternion Integration
 */

#include "ahrs_hw.h"
#include <stdint.h>
#include <stdio.h>  // ADDED FOR DEBUG PRINTS

// Union for Fast Inverse Sqrt (bit manipulation)
typedef union {
    float f;
    int32_t i;
} Union32;

//==============================================================================
// HELPER FUNCTION: Fast Inverse Square Root
//==============================================================================

static inline float fast_inv_sqrt(float x) {
    Union32 u = {.f = x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

//==============================================================================
// MAIN HW FUNCTION WITH FOR LOOP
//==============================================================================

void AhrsHW_ProcessBatch(
    const HwInputSample inputs[],
    HwOutputSample outputs[],
    int numSamples,
    FusionAhrs *const ahrs
) {
    // Initial quaternion (read from AHRS state)
    float qw = ahrs->quaternion.element.w;
    float qx = ahrs->quaternion.element.x;
    float qy = ahrs->quaternion.element.y;
    float qz = ahrs->quaternion.element.z;
    
    // DEBUG PRINT - START
    printf("\n=== HW BATCH START ===\n");
    printf("numSamples: %d\n", numSamples);
    printf("Initial qw=%.6f, qx=%.6f, qy=%.6f, qz=%.6f\n\n", qw, qx, qy, qz);
    
    //==========================================================================
    // FOR LOOP - Processes all samples in the batch
    // 
    // In VHDL: This becomes an FSM with states:
    //   IDLE -> LOAD_SAMPLE -> SECTION5 -> SECTION6 -> SECTION7 -> SECTION8 -> 
    //   STORE_OUTPUT -> (back to LOAD_SAMPLE or DONE)
    //==========================================================================
    
    for (int i = 0; i < numSamples; i++) {
        
        // DEBUG PRINT - SAMPLE START
        printf("--- SAMPLE %d ---\n", i+1);
        
        // Load input data for this sample
        const float gx = inputs[i].gx;
        const float gy = inputs[i].gy;
        const float gz = inputs[i].gz;
        const float ax = inputs[i].ax;
        const float ay = inputs[i].ay;
        const float az = inputs[i].az;
        const float halfGravityX = inputs[i].halfGravityX;
        const float halfGravityY = inputs[i].halfGravityY;
        const float halfGravityZ = inputs[i].halfGravityZ;
        const float deltaTime = inputs[i].deltaTime;
        const float rampedGain = inputs[i].rampedGain;
        
        // DEBUG PRINT - INPUT
        printf("INPUT: gx=%.6f, gy=%.6f, gz=%.6f\n", gx, gy, gz);
        printf("       ax=%.6f, ay=%.6f, az=%.6f\n", ax, ay, az);
        printf("       hg=[%.6f, %.6f, %.6f], dt=%.6f, gain=%.6f\n", 
               halfGravityX, halfGravityY, halfGravityZ, deltaTime, rampedGain);
        
        //======================================================================
        // SECTION 5: ACCELEROMETER FEEDBACK
        //======================================================================
        
        printf("\n[SECTION 5: ACCEL FEEDBACK]\n");
        
        float halfAccelFeedbackX = 0.0f;
        float halfAccelFeedbackY = 0.0f;
        float halfAccelFeedbackZ = 0.0f;
        
        // Check if accelerometer != 0
        const bool accelNotZero = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
        printf("  accelNotZero: %d\n", accelNotZero);
        
        if (accelNotZero) {
            
            // Magnitude squared
            const float accelMagSquared = ax * ax + ay * ay + az * az;
            printf("  accelMagSquared: %.6f\n", accelMagSquared);
            
            // FAST INVERSE SQRT #1 - accelerometer normalization
            const float accelInvMag = fast_inv_sqrt(accelMagSquared);
            printf("  accelInvMag (INV_SQRT #1): %.6f\n", accelInvMag);
            
            // Normalized accelerometer
            const float normAccelX = ax * accelInvMag;
            const float normAccelY = ay * accelInvMag;
            const float normAccelZ = az * accelInvMag;
            printf("  normAccel: [%.6f, %.6f, %.6f]\n", normAccelX, normAccelY, normAccelZ);
            
            // Cross product: normAccel × halfGravity
            float crossX = normAccelY * halfGravityZ - normAccelZ * halfGravityY;
            float crossY = normAccelZ * halfGravityX - normAccelX * halfGravityZ;
            float crossZ = normAccelX * halfGravityY - normAccelY * halfGravityX;
            printf("  cross product: [%.6f, %.6f, %.6f]\n", crossX, crossY, crossZ);
            
            // Dot product for orientation check
            const float dotProduct = normAccelX * halfGravityX + 
                                     normAccelY * halfGravityY + 
                                     normAccelZ * halfGravityZ;
            printf("  dotProduct: %.6f (< 0? %d)\n", dotProduct, dotProduct < 0.0f);
            
            // If dot < 0, normalize cross product
            if (dotProduct < 0.0f) {
                const float crossMagSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;
                printf("  crossMagSquared: %.6f\n", crossMagSquared);
                
                // FAST INVERSE SQRT #2 - cross product normalization
                const float crossInvMag = fast_inv_sqrt(crossMagSquared);
                printf("  crossInvMag (INV_SQRT #2): %.6f\n", crossInvMag);
                
                crossX *= crossInvMag;
                crossY *= crossInvMag;
                crossZ *= crossInvMag;
                printf("  normalized cross: [%.6f, %.6f, %.6f]\n", crossX, crossY, crossZ);
            }
            
            // Apply feedback
            halfAccelFeedbackX = crossX;
            halfAccelFeedbackY = crossY;
            halfAccelFeedbackZ = crossZ;
        }
        
        printf("  halfAccelFeedback: [%.6f, %.6f, %.6f]\n", 
               halfAccelFeedbackX, halfAccelFeedbackY, halfAccelFeedbackZ);
        
        //======================================================================
        // SECTION 6: GYROSCOPE CONVERSION
        // Conversion: deg/s -> rad/s, multiplied by 0.5 for quaternion math
        //======================================================================
        
        printf("\n[SECTION 6: GYRO CONVERSION]\n");
        
        // Original Fusion: FusionDegreesToRadians(0.5f) = 0.5 * PI/180 = PI/360
        const float degToRadHalf = M_PI / 360.0f;
        
        const float halfGyroX = gx * degToRadHalf;
        const float halfGyroY = gy * degToRadHalf;
        const float halfGyroZ = gz * degToRadHalf;
        
        printf("  degToRadHalf: %.10f\n", degToRadHalf);
        printf("  halfGyro: [%.6f, %.6f, %.6f]\n", halfGyroX, halfGyroY, halfGyroZ);
        
        //======================================================================
        // SECTION 7: APPLY FEEDBACK
        // Combine gyro with accelerometer feedback
        //======================================================================
        
        printf("\n[SECTION 7: APPLY FEEDBACK]\n");
        
        const float adjHalfGyroX = halfGyroX + halfAccelFeedbackX * rampedGain;
        const float adjHalfGyroY = halfGyroY + halfAccelFeedbackY * rampedGain;
        const float adjHalfGyroZ = halfGyroZ + halfAccelFeedbackZ * rampedGain;
        
        printf("  rampedGain: %.6f\n", rampedGain);
        printf("  adjHalfGyro: [%.6f, %.6f, %.6f]\n", adjHalfGyroX, adjHalfGyroY, adjHalfGyroZ);
        
        //======================================================================
        // SECTION 8: QUATERNION INTEGRATION
        // q_new = q + dq * dt
        //======================================================================
        
        printf("\n[SECTION 8: QUAT INTEGRATION]\n");
        printf("  current q: [%.6f, %.6f, %.6f, %.6f]\n", qw, qx, qy, qz);
        
        // Quaternion derivative
        const float dqw = -qx * adjHalfGyroX - qy * adjHalfGyroY - qz * adjHalfGyroZ;
        const float dqx =  qw * adjHalfGyroX + qy * adjHalfGyroZ - qz * adjHalfGyroY;
        const float dqy =  qw * adjHalfGyroY - qx * adjHalfGyroZ + qz * adjHalfGyroX;
        const float dqz =  qw * adjHalfGyroZ + qx * adjHalfGyroY - qy * adjHalfGyroX;
        
        printf("  dq: [%.6f, %.6f, %.6f, %.6f]\n", dqw, dqx, dqy, dqz);
        
        // Euler integration
        const float newQw = qw + dqw * deltaTime;
        const float newQx = qx + dqx * deltaTime;
        const float newQy = qy + dqy * deltaTime;
        const float newQz = qz + dqz * deltaTime;
        
        printf("  deltaTime: %.6f\n", deltaTime);
        printf("  new q: [%.6f, %.6f, %.6f, %.6f]\n", newQw, newQx, newQy, newQz);
        
        //======================================================================
        // STORE OUTPUT - Save result for this sample
        //======================================================================
        
        outputs[i].qw = newQw;
        outputs[i].qx = newQx;
        outputs[i].qy = newQy;
        outputs[i].qz = newQz;
        
        printf("  OUTPUT[%d]: [%.6f, %.6f, %.6f, %.6f]\n\n", i, newQw, newQx, newQy, newQz);
        
        //======================================================================
        // UPDATE STATE - Prepare for next iteration
        // Quaternion from this iteration becomes input for the next
        //======================================================================
        
        qw = newQw;
        qx = newQx;
        qy = newQy;
        qz = newQz;
    }
    
    // DEBUG PRINT - END
    printf("=== HW BATCH END ===\n\n");
    
    //==========================================================================
    // END OF FOR LOOP
    // Final quaternion (unnormalized) remains in outputs[numSamples-1]
    //==========================================================================
}

//==============================================================================
// ALTERNATIVE HW FUNCTION FOR SINGLE SAMPLE
// Used by main.c for per-sample processing
//==============================================================================

void AhrsHW_Sections5to8(
    FusionAhrs *const ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float halfGravityX, float halfGravityY, float halfGravityZ,
    float deltaTime,
    float *newQw, float *newQx, float *newQy, float *newQz
) {
    // Current quaternion
    float qw = ahrs->quaternion.element.w;
    float qx = ahrs->quaternion.element.x;
    float qy = ahrs->quaternion.element.y;
    float qz = ahrs->quaternion.element.z;
    
    //==========================================================================
    // SECTION 5: ACCELEROMETER FEEDBACK
    //==========================================================================
    
    float halfAccelFeedbackX = 0.0f;
    float halfAccelFeedbackY = 0.0f;
    float halfAccelFeedbackZ = 0.0f;
    
    // Check if accelerometer != 0
    const bool accelNotZero = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
    
    if (accelNotZero) {
        
        // Magnitude squared
        const float accelMagSquared = ax * ax + ay * ay + az * az;
        
        // FAST INVERSE SQRT #1 - accelerometer normalization
        const float accelInvMag = fast_inv_sqrt(accelMagSquared);
        
        // Normalized accelerometer
        const float normAccelX = ax * accelInvMag;
        const float normAccelY = ay * accelInvMag;
        const float normAccelZ = az * accelInvMag;
        
        // Cross product: normAccel × halfGravity
        float crossX = normAccelY * halfGravityZ - normAccelZ * halfGravityY;
        float crossY = normAccelZ * halfGravityX - normAccelX * halfGravityZ;
        float crossZ = normAccelX * halfGravityY - normAccelY * halfGravityX;
        
        // Dot product for orientation check
        const float dotProduct = normAccelX * halfGravityX + 
                                 normAccelY * halfGravityY + 
                                 normAccelZ * halfGravityZ;
        
        // If dot < 0, normalize cross product
        if (dotProduct < 0.0f) {
            const float crossMagSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;
            
            // FAST INVERSE SQRT #2 - cross product normalization
            const float crossInvMag = fast_inv_sqrt(crossMagSquared);
            
            crossX *= crossInvMag;
            crossY *= crossInvMag;
            crossZ *= crossInvMag;
        }
        
        // Apply feedback
        halfAccelFeedbackX = crossX;
        halfAccelFeedbackY = crossY;
        halfAccelFeedbackZ = crossZ;
    }
    
    //==========================================================================
    // SECTION 6: GYROSCOPE CONVERSION
    // Conversion: deg/s -> rad/s, multiplied by 0.5 for quaternion math
    //==========================================================================
    
    const float degToRadHalf = M_PI / 360.0f;  // = (PI/180) * 0.5
    
    const float halfGyroX = gx * degToRadHalf;
    const float halfGyroY = gy * degToRadHalf;
    const float halfGyroZ = gz * degToRadHalf;
    
    //==========================================================================
    // SECTION 7: APPLY FEEDBACK
    // Combine gyro with accelerometer feedback
    //==========================================================================
    
    const float rampedGain = ahrs->rampedGain;
    
    const float adjHalfGyroX = halfGyroX + halfAccelFeedbackX * rampedGain;
    const float adjHalfGyroY = halfGyroY + halfAccelFeedbackY * rampedGain;
    const float adjHalfGyroZ = halfGyroZ + halfAccelFeedbackZ * rampedGain;
    
    //==========================================================================
    // SECTION 8: QUATERNION INTEGRATION
    // q_new = q + dq * dt
    //==========================================================================
    
    // Quaternion derivative
    const float dqw = -qx * adjHalfGyroX - qy * adjHalfGyroY - qz * adjHalfGyroZ;
    const float dqx =  qw * adjHalfGyroX + qy * adjHalfGyroZ - qz * adjHalfGyroY;
    const float dqy =  qw * adjHalfGyroY - qx * adjHalfGyroZ + qz * adjHalfGyroX;
    const float dqz =  qw * adjHalfGyroZ + qx * adjHalfGyroY - qy * adjHalfGyroX;
    
    // Euler integration
    *newQw = qw + dqw * deltaTime;
    *newQx = qx + dqx * deltaTime;
    *newQy = qy + dqy * deltaTime;
    *newQz = qz + dqz * deltaTime;
}
