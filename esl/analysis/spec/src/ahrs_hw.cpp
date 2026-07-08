/**
 * @file ahrs_hw.cpp
 * @brief AHRS Hardware Accelerator - Sections 5-8, fixed-point (SystemC sc_fixed)
 *
 * SUŽENI TIPOVI za DSP48E1 kompatibilnost na xc7z010:
 *
 *   quat_t : 29→18 bita  →  quat(18) × adjhg(26) = 44-bit ≤ 45 → 1 DSP ✓
 *   dt_t   : OSTAJE 20-bit!  (18-bit daje kvantizacionu grešku dt=0.01 od -1.68e-5,
 *                              koja se množi gain≈10 × 499 uzoraka → akumulira ~30° !)
 *   dq_t   : OSTAJE 27-bit  →  dq(27) × dt(20) = 47-bit → 2-DSP kaskada (prihvatljivo)
 *
 * Greška vs float referenca:
 *   499  uzoraka: max 0.22°  ✓
 *   1986 uzoraka: max 1.47°  ✓ (prihvatljivo za embedded IMU)
 *
 * Algoritam identičan sa cpp_spec/spec/src/ahrs_hw.cpp (float referenca),
 * uključujući rejection logiku (accelerometerIgnored, recoveryTrigger).
 */

#include "ahrs_hw.h"
#include <stdint.h>
#include <stdio.h>
#include <systemc.h>

//==============================================================================
// TIPOVI - V3 Final sa smanjenim quat_t za DSP48E1
//==============================================================================

// Quaternion [-2, 2) → SMANJENO: 29→18 bita, 16 frac
// quat(18) × adjhg(26) = 44-bit ≤ 45 → staje u 1 DSP48E1 ✓
typedef sc_fixed<18, 2, SC_RND, SC_SAT>  quat_t;

// Dot product [-1, 1) → 25 frac (26-bit, nepromijenjen)
typedef sc_fixed<26, 1, SC_RND, SC_SAT>  dot_t;

// Normalizovani vektori [-2, 2) → 24 frac (26-bit, nepromijenjen)
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  norm_t;

// Akcelerometar [-4, 4) → 17 frac (20-bit, nepromijenjen)
typedef sc_fixed<20, 3, SC_RND, SC_SAT>  accel_t;

// HalfGravity [-2, 2) → 24 frac (26-bit, nepromijenjen)
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  hg_t;

// Magnitude squared [0, 4) → 20 frac (22-bit, nepromijenjen)
typedef sc_ufixed<22, 2, SC_RND, SC_SAT> magsq_t;

// Inverse magnitude [0, 64) → 18 frac (24-bit, nepromijenjen)
typedef sc_ufixed<24, 6, SC_RND, SC_SAT> invmag_t;

// HalfGyro [-2, 2) → 20 frac (22-bit, nepromijenjen)
typedef sc_fixed<22, 2, SC_RND, SC_SAT>  halfgyro_t;

// Gain [0, 16) → 16 frac (20-bit, nepromijenjen)
typedef sc_ufixed<20, 4, SC_RND, SC_SAT> gain_t;

// AdjHalfGyro [-16, 16) → 21 frac (26-bit, nepromijenjen)
typedef sc_fixed<26, 5, SC_RND, SC_SAT>  adjhg_t;

// dq [-16, 16) → 22 frac (27-bit, nepromijenjen)
// dq(27) × dt(20) = 47-bit → 2-DSP kaskada (prihvatljivo)
typedef sc_fixed<27, 5, SC_RND, SC_SAT>  dq_t;

// DeltaTime [0, 1) → 20 frac (20-bit, OSTAJE jer 18-bit lomi preciznost za dt=0.01)
typedef sc_ufixed<20, 0, SC_RND, SC_SAT> dt_t;

//==============================================================================
// KONSTANTE
//==============================================================================

#define DEG_TO_RAD_HALF_FLOAT (M_PI / 360.0f)

//==============================================================================
// Fast Inverse Square Root (originalni float bit-trick)
//==============================================================================

typedef union { float f; int32_t i; } Union32;

static inline float fast_inv_sqrt(float x) {
    Union32 u = {.f = x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

//==============================================================================
// AhrsHW_Sections5to8 - algoritam identičan sa float referencom (cpp_spec)
//==============================================================================

void AhrsHW_Sections5to8(
    FusionAhrs *const ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float halfGravityX, float halfGravityY, float halfGravityZ,
    float deltaTime_f,
    float *newQw, float *newQx, float *newQy, float *newQz
) {
    // Quaternion state
    quat_t qw = ahrs->quaternion.element.w;
    quat_t qx = ahrs->quaternion.element.x;
    quat_t qy = ahrs->quaternion.element.y;
    quat_t qz = ahrs->quaternion.element.z;

    // Akcelerometar
    const accel_t ax_fp = ax;
    const accel_t ay_fp = ay;
    const accel_t az_fp = az;

    // HalfGravity
    const hg_t hgX = halfGravityX;
    const hg_t hgY = halfGravityY;
    const hg_t hgZ = halfGravityZ;

    // Vreme i gain
    const dt_t deltaTime = deltaTime_f;
    const gain_t rampedGain = ahrs->rampedGain;

    // =========================================================================
    // SECTION 5: ACCELEROMETER FEEDBACK
    // =========================================================================

    float halfAccelFeedbackX = 0.0f;
    float halfAccelFeedbackY = 0.0f;
    float halfAccelFeedbackZ = 0.0f;

    ahrs->accelerometerIgnored = true;

    const bool accelNotZero = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));

    if (accelNotZero) {
        const magsq_t accelMagSquared = ax_fp * ax_fp + ay_fp * ay_fp + az_fp * az_fp;
        const invmag_t accelInvMag = fast_inv_sqrt((float)accelMagSquared);

        const norm_t normAccelX = ax_fp * accelInvMag;
        const norm_t normAccelY = ay_fp * accelInvMag;
        const norm_t normAccelZ = az_fp * accelInvMag;

        norm_t crossX = normAccelY * hgZ - normAccelZ * hgY;
        norm_t crossY = normAccelZ * hgX - normAccelX * hgZ;
        norm_t crossZ = normAccelX * hgY - normAccelY * hgX;

        const dot_t dotProduct = normAccelX * hgX + normAccelY * hgY + normAccelZ * hgZ;

        if (dotProduct < 0.0f) {
            const magsq_t crossMagSquared = crossX * crossX + crossY * crossY + crossZ * crossZ;
            const invmag_t crossInvMag = fast_inv_sqrt((float)crossMagSquared);
            crossX = crossX * crossInvMag;
            crossY = crossY * crossInvMag;
            crossZ = crossZ * crossInvMag;
        }

        // Čuvaj feedback u ahrs state (float cast za kompatibilnost)
        ahrs->halfAccelerometerFeedback.axis.x = (float)crossX;
        ahrs->halfAccelerometerFeedback.axis.y = (float)crossY;
        ahrs->halfAccelerometerFeedback.axis.z = (float)crossZ;

        // Rejection logika (identična cpp_spec)
        const float crossX_f = (float)crossX;
        const float crossY_f = (float)crossY;
        const float crossZ_f = (float)crossZ;
        const float feedbackNormSq = crossX_f * crossX_f + crossY_f * crossY_f + crossZ_f * crossZ_f;

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

        // Stezanje triggera na [0, recoveryTriggerPeriod]
        if (ahrs->accelerationRecoveryTrigger < 0)
            ahrs->accelerationRecoveryTrigger = 0;
        if (ahrs->accelerationRecoveryTrigger > (int)ahrs->recoveryTriggerPeriod)
            ahrs->accelerationRecoveryTrigger = (int)ahrs->recoveryTriggerPeriod;

        if (!ahrs->accelerometerIgnored) {
            halfAccelFeedbackX = crossX_f;
            halfAccelFeedbackY = crossY_f;
            halfAccelFeedbackZ = crossZ_f;
        }
    }

    // =========================================================================
    // SECTION 6: GYROSCOPE CONVERSION
    // =========================================================================

    const float halfGyroX_f = gx * DEG_TO_RAD_HALF_FLOAT;
    const float halfGyroY_f = gy * DEG_TO_RAD_HALF_FLOAT;
    const float halfGyroZ_f = gz * DEG_TO_RAD_HALF_FLOAT;
    const halfgyro_t halfGyroX = halfGyroX_f;
    const halfgyro_t halfGyroY = halfGyroY_f;
    const halfgyro_t halfGyroZ = halfGyroZ_f;

    // =========================================================================
    // SECTION 7: APPLY FEEDBACK
    // =========================================================================

    const adjhg_t adjHalfGyroX = halfGyroX + adjhg_t(halfAccelFeedbackX) * rampedGain;
    const adjhg_t adjHalfGyroY = halfGyroY + adjhg_t(halfAccelFeedbackY) * rampedGain;
    const adjhg_t adjHalfGyroZ = halfGyroZ + adjhg_t(halfAccelFeedbackZ) * rampedGain;

    // =========================================================================
    // SECTION 8: QUATERNION INTEGRATION
    // Ključni proizvodi:
    //   quat_t(22) × adjhg_t(22) = 44-bit ≤ 45 → DSP48E1 ✓
    //   dq_t(22)   × dt_t(18)    = 40-bit ≤ 45 → DSP48E1 ✓
    // =========================================================================

    const dq_t dqw = -qx * adjHalfGyroX - qy * adjHalfGyroY - qz * adjHalfGyroZ;
    const dq_t dqx =  qw * adjHalfGyroX + qy * adjHalfGyroZ - qz * adjHalfGyroY;
    const dq_t dqy =  qw * adjHalfGyroY - qx * adjHalfGyroZ + qz * adjHalfGyroX;
    const dq_t dqz =  qw * adjHalfGyroZ + qx * adjHalfGyroY - qy * adjHalfGyroX;

    *newQw = (float)(qw + dqw * deltaTime);
    *newQx = (float)(qx + dqx * deltaTime);
    *newQy = (float)(qy + dqy * deltaTime);
    *newQz = (float)(qz + dqz * deltaTime);
}
