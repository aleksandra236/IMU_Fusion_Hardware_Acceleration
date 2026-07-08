/**
 * @file ahrs_hw.cpp
 * @brief AHRS Hardware Accelerator - Sections 5-8, float, sample-by-sample
 *
 * Float referenca (ground truth) za poređenje sa fixed-point implementacijom.
 * Samo AhrsHW_Sections5to8 - nema batch procesiranja.
 */

#include "ahrs_hw.h"
#include <stdint.h>
#include <math.h>

// Union for Fast Inverse Sqrt
typedef union { float f; int32_t i; } Union32;

static inline float fast_inv_sqrt(float x) {
    Union32 u = {.f = x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

void AhrsHW_Sections5to8(
    FusionAhrs *const ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float halfGravityX, float halfGravityY, float halfGravityZ,
    float deltaTime,
    float *newQw, float *newQx, float *newQy, float *newQz
) {
    const float qw = ahrs->quaternion.element.w;
    const float qx = ahrs->quaternion.element.x;
    const float qy = ahrs->quaternion.element.y;
    const float qz = ahrs->quaternion.element.z;

    // SECTION 5: ACCELEROMETER FEEDBACK
    float halfAccelFeedbackX = 0.0f;
    float halfAccelFeedbackY = 0.0f;
    float halfAccelFeedbackZ = 0.0f;

    ahrs->accelerometerIgnored = true;

    const bool accelNotZero = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));

    if (accelNotZero) {
        const float accelMagSquared = ax*ax + ay*ay + az*az;
        const float accelInvMag = fast_inv_sqrt(accelMagSquared);

        const float normAccelX = ax * accelInvMag;
        const float normAccelY = ay * accelInvMag;
        const float normAccelZ = az * accelInvMag;

        float crossX = normAccelY * halfGravityZ - normAccelZ * halfGravityY;
        float crossY = normAccelZ * halfGravityX - normAccelX * halfGravityZ;
        float crossZ = normAccelX * halfGravityY - normAccelY * halfGravityX;

        const float dotProduct = normAccelX * halfGravityX +
                                 normAccelY * halfGravityY +
                                 normAccelZ * halfGravityZ;

        if (dotProduct < 0.0f) {
            const float crossMagSquared = crossX*crossX + crossY*crossY + crossZ*crossZ;
            const float crossInvMag = fast_inv_sqrt(crossMagSquared);
            crossX *= crossInvMag;
            crossY *= crossInvMag;
            crossZ *= crossInvMag;
        }

        ahrs->halfAccelerometerFeedback.axis.x = crossX;
        ahrs->halfAccelerometerFeedback.axis.y = crossY;
        ahrs->halfAccelerometerFeedback.axis.z = crossZ;

        // Rejection logic: ignore accelerometer if error exceeds threshold
        const float feedbackNormSq = crossX*crossX + crossY*crossY + crossZ*crossZ;
        if (ahrs->initialising || (feedbackNormSq <= ahrs->accelerationRejection)) {
            ahrs->accelerometerIgnored = false;
            ahrs->accelerationRecoveryTrigger -= 9;
        } else {
            ahrs->accelerationRecoveryTrigger += 1;
        }

        if (ahrs->accelerationRecoveryTrigger > ahrs->accelerationRecoveryTimeout) {
            ahrs->accelerationRecoveryTimeout = 0;
            ahrs->accelerometerIgnored = false;
        } else {
            ahrs->accelerationRecoveryTimeout = (int)ahrs->recoveryTriggerPeriod;
        }

        // Clamp trigger to [0, recoveryTriggerPeriod]
        if (ahrs->accelerationRecoveryTrigger < 0)
            ahrs->accelerationRecoveryTrigger = 0;
        if (ahrs->accelerationRecoveryTrigger > (int)ahrs->recoveryTriggerPeriod)
            ahrs->accelerationRecoveryTrigger = (int)ahrs->recoveryTriggerPeriod;

        if (!ahrs->accelerometerIgnored) {
            halfAccelFeedbackX = crossX;
            halfAccelFeedbackY = crossY;
            halfAccelFeedbackZ = crossZ;
        }
    }

    // SECTION 6: GYROSCOPE CONVERSION
    const float degToRadHalf = (float)M_PI / 360.0f;
    const float halfGyroX = gx * degToRadHalf;
    const float halfGyroY = gy * degToRadHalf;
    const float halfGyroZ = gz * degToRadHalf;

    // SECTION 7: APPLY FEEDBACK
    const float rampedGain = ahrs->rampedGain;
    const float adjHalfGyroX = halfGyroX + halfAccelFeedbackX * rampedGain;
    const float adjHalfGyroY = halfGyroY + halfAccelFeedbackY * rampedGain;
    const float adjHalfGyroZ = halfGyroZ + halfAccelFeedbackZ * rampedGain;

    // SECTION 8: QUATERNION INTEGRATION
    const float dqw = -qx * adjHalfGyroX - qy * adjHalfGyroY - qz * adjHalfGyroZ;
    const float dqx =  qw * adjHalfGyroX + qy * adjHalfGyroZ - qz * adjHalfGyroY;
    const float dqy =  qw * adjHalfGyroY - qx * adjHalfGyroZ + qz * adjHalfGyroX;
    const float dqz =  qw * adjHalfGyroZ + qx * adjHalfGyroY - qy * adjHalfGyroX;

    *newQw = qw + dqw * deltaTime;
    *newQx = qx + dqx * deltaTime;
    *newQy = qy + dqy * deltaTime;
    *newQz = qz + dqz * deltaTime;
}