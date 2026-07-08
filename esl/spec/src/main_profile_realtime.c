/**
 * @file main_profile_realtime.c
 * @brief Real-time profiling version — measures per-section wall-clock timing.
 *
 * Processes one sample at a time (same as main.c / main_embedded.c).
 * Uses real sensor data compiled in from sensor_data_embedded.h (zero file I/O).
 *
 * Each of the 11 algorithmic sections is wrapped with clock_gettime() calls so
 * that accumulated time per section can be printed at the end. This lets us see
 * which sections dominate execution time without Callgrind instruction counting.
 *
 * USAGE:
 *   make ahrs_profile_rt
 *   ./ahrs_profile_rt
 */

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include "ahrs_hw.h"
#include "sensor_data_embedded.h"

#define SAMPLE_RATE 100

typedef union { float f; int32_t i; } Union32;

//------------------------------------------------------------------------------
// Timing helpers
//------------------------------------------------------------------------------

typedef struct {
    long long sec1_4_ns;   /* SW: Sections 1-4   (pre-HW)  */
    long long sec5_8_ns;   /* HW: Sections 5-8              */
    long long sec9_11_ns;  /* SW: Sections 9-11  (post-HW) */
} SectionTimes;

static inline long long timespec_ns(struct timespec t) {
    return (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
}

//------------------------------------------------------------------------------
// Initialisation
//------------------------------------------------------------------------------

static void ahrs_init(FusionAhrs *const ahrs) {
    memset(ahrs, 0, sizeof(FusionAhrs));
    ahrs->quaternion     = FUSION_IDENTITY_QUATERNION;
    ahrs->initialising   = true;
    ahrs->rampedGain     = 10.0f;
    ahrs->gain           = 0.5f;
    ahrs->gyroscopeRange = 2000.0f;
    ahrs->rampedGainStep = (10.0f - 0.5f) / 3.0f;
}

//------------------------------------------------------------------------------
// Process one sample, accumulate timing per section group
//------------------------------------------------------------------------------

static void process_one_sample(FusionAhrs *const ahrs,
                               float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float deltaTime,
                               SectionTimes *times)
{
    struct timespec t0, t1, t2, t3;

    /* ── SW: Sections 1-4 ─────────────────────────────────────────────────── */
    clock_gettime(CLOCK_MONOTONIC, &t0);

    /* Section 1 – store accelerometer */
    ahrs->accelerometer.axis.x = ax;
    ahrs->accelerometer.axis.y = ay;
    ahrs->accelerometer.axis.z = az;

    /* Section 2 – gyroscope range check */
    if ((fabsf(gx) > ahrs->gyroscopeRange) ||
        (fabsf(gy) > ahrs->gyroscopeRange) ||
        (fabsf(gz) > ahrs->gyroscopeRange)) {
        ahrs->angularRateRecovery = true;
    }

    /* Section 3 – ramp gain */
    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if ((ahrs->rampedGain < ahrs->gain) || (ahrs->gain == 0.0f)) {
            ahrs->rampedGain          = ahrs->gain;
            ahrs->initialising        = false;
            ahrs->angularRateRecovery = false;
        }
    }

    /* Section 4 – half gravity vector */
    const float qw = ahrs->quaternion.element.w;
    const float qx = ahrs->quaternion.element.x;
    const float qy = ahrs->quaternion.element.y;
    const float qz = ahrs->quaternion.element.z;

    const float halfGravityX = qx * qz - qw * qy;
    const float halfGravityY = qy * qz + qw * qx;
    const float halfGravityZ = qw * qw - 0.5f + qz * qz;

    clock_gettime(CLOCK_MONOTONIC, &t1);

    /* ── HW: Sections 5-8 ─────────────────────────────────────────────────── */
    float newQw, newQx, newQy, newQz;
    AhrsHW_Sections5to8(ahrs,
                        gx, gy, gz,
                        ax, ay, az,
                        halfGravityX, halfGravityY, halfGravityZ,
                        deltaTime,
                        &newQw, &newQx, &newQy, &newQz);

    clock_gettime(CLOCK_MONOTONIC, &t2);

    /* ── SW: Sections 9-11 ────────────────────────────────────────────────── */

    /* Section 9 – fast-inv-sqrt normalisation */
    const float magSq = newQw*newQw + newQx*newQx + newQy*newQy + newQz*newQz;
    Union32 u = { .f = magSq };
    u.i = 0x5F1F1412 - (u.i >> 1);
    const float invMag = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);

    ahrs->quaternion.element.w = newQw * invMag;
    ahrs->quaternion.element.x = newQx * invMag;
    ahrs->quaternion.element.y = newQy * invMag;
    ahrs->quaternion.element.z = newQz * invMag;

    /* Section 10 – zero heading during initialisation */
    if (ahrs->initialising) {
        const float yaw = atan2f(
            ahrs->quaternion.element.w * ahrs->quaternion.element.z +
            ahrs->quaternion.element.x * ahrs->quaternion.element.y,
            0.5f - ahrs->quaternion.element.y * ahrs->quaternion.element.y -
            ahrs->quaternion.element.z * ahrs->quaternion.element.z);

        const float half = 0.5f * yaw;
        const float cw   =  cosf(half);
        const float sz   = -sinf(half);

        const float rw = ahrs->quaternion.element.w;
        const float rx = ahrs->quaternion.element.x;
        const float ry = ahrs->quaternion.element.y;
        const float rz = ahrs->quaternion.element.z;

        ahrs->quaternion.element.w = cw * rw - sz * rz;
        ahrs->quaternion.element.x = cw * rx - sz * ry;
        ahrs->quaternion.element.y = cw * ry + sz * rx;
        ahrs->quaternion.element.z = cw * rz + sz * rw;
    }

    /* Section 11 – quaternion updated in ahrs state (caller reads it) */

    clock_gettime(CLOCK_MONOTONIC, &t3);

    times->sec1_4_ns  += timespec_ns(t1) - timespec_ns(t0);
    times->sec5_8_ns  += timespec_ns(t2) - timespec_ns(t1);
    times->sec9_11_ns += timespec_ns(t3) - timespec_ns(t2);
}

//------------------------------------------------------------------------------
// Main
//------------------------------------------------------------------------------

int main(void) {
    printf("Real-time profiling — %d samples, sample-by-sample\n\n",
           EMBEDDED_NUM_SAMPLES);

    FusionAhrs ahrs;
    ahrs_init(&ahrs);

    SectionTimes times = {0, 0, 0};
    const float deltaTime = 1.0f / SAMPLE_RATE;

    for (int i = 0; i < EMBEDDED_NUM_SAMPLES; i++) {
        process_one_sample(&ahrs,
                           SENSOR_DATA[i].gx, SENSOR_DATA[i].gy, SENSOR_DATA[i].gz,
                           SENSOR_DATA[i].ax, SENSOR_DATA[i].ay, SENSOR_DATA[i].az,
                           deltaTime,
                           &times);
    }

    long long total_ns = times.sec1_4_ns + times.sec5_8_ns + times.sec9_11_ns;

    printf("=== Per-section timing (%d samples) ===\n", EMBEDDED_NUM_SAMPLES);
    printf("  SW Sec 1-4  : %8.3f us  (%5.1f%%)\n",
           times.sec1_4_ns  / 1000.0, 100.0 * times.sec1_4_ns  / total_ns);
    printf("  HW Sec 5-8  : %8.3f us  (%5.1f%%)\n",
           times.sec5_8_ns  / 1000.0, 100.0 * times.sec5_8_ns  / total_ns);
    printf("  SW Sec 9-11 : %8.3f us  (%5.1f%%)\n",
           times.sec9_11_ns / 1000.0, 100.0 * times.sec9_11_ns / total_ns);
    printf("  Total       : %8.3f us\n", total_ns / 1000.0);
    printf("  Per sample  : %8.3f us\n", total_ns / 1000.0 / EMBEDDED_NUM_SAMPLES);

    printf("\nFinal orientation:\n");
    printf("  Q = [%.6f, %.6f, %.6f, %.6f]\n",
           ahrs.quaternion.element.w, ahrs.quaternion.element.x,
           ahrs.quaternion.element.y, ahrs.quaternion.element.z);

    return 0;
}