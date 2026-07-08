/**
 * @file main_embedded.c
 * @brief Original FusionAhrs algorithm – one sample at a time, no batching.
 *
 * Mirrors FusionAhrsUpdateNoMagnetometer() from the upstream Fusion library:
 *   for each sample:
 *     Sec 1  – store accelerometer
 *     Sec 2  – gyroscope range check
 *     Sec 3  – ramp gain
 *     Sec 4  – halfGravity from CURRENT (just-normalised) quaternion  ← fresh every sample
 *     Sec 5-8 – HW accelerator call  (AhrsHW_Sections5to8)
 *     Sec 9  – normalise quaternion
 *     Sec 10 – zero heading (during initialisation)
 *     Sec 11 – output
 *
 * All 1986 sensor samples are in SENSOR_DATA[] (sensor_data_embedded.h).
 * Zero file I/O at run-time → profiling captures only algorithmic cost.
 *
 * USAGE:
 *   make ahrs_embedded
 *   ./ahrs_embedded                  # no output file, pure compute
 *   ./ahrs_embedded out.csv          # optional: write orientation CSV
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "ahrs_hw.h"
#include "sensor_data_embedded.h"   /* SENSOR_DATA[], EMBEDDED_NUM_SAMPLES */

/*-----------------------------------------------------------------------------
 * Configuration
 *---------------------------------------------------------------------------*/
#define SAMPLE_RATE  100

/*-----------------------------------------------------------------------------
 * Union for fast inverse sqrt
 *---------------------------------------------------------------------------*/
typedef union { float f; int32_t i; } Union32;

/*-----------------------------------------------------------------------------
 * Euler helper
 *---------------------------------------------------------------------------*/
typedef struct { struct { float roll, pitch, yaw; } angle; } LocalEuler;

static LocalEuler euler_from_quat(const FusionQuaternion q)
{
    LocalEuler e;
    e.angle.roll  = atan2f(q.element.w * q.element.x + q.element.y * q.element.z,
                           0.5f - q.element.y * q.element.y - q.element.x * q.element.x)
                    * (180.0f / M_PI);
    e.angle.pitch = asinf(2.0f * (q.element.w * q.element.y - q.element.z * q.element.x))
                    * (180.0f / M_PI);
    e.angle.yaw   = atan2f(q.element.w * q.element.z + q.element.x * q.element.y,
                           0.5f - q.element.y * q.element.y - q.element.z * q.element.z)
                    * (180.0f / M_PI);
    return e;
}

/*-----------------------------------------------------------------------------
 * Initialisation
 *---------------------------------------------------------------------------*/
static void ahrs_init(FusionAhrs *const ahrs)
{
    memset(ahrs, 0, sizeof(FusionAhrs));
    ahrs->quaternion        = FUSION_IDENTITY_QUATERNION;
    ahrs->initialising      = true;
    ahrs->rampedGain        = 10.0f;
    ahrs->gain              = 0.5f;
    ahrs->gyroscopeRange    = 2000.0f;
    ahrs->rampedGainStep    = (10.0f - 0.5f) / 3.0f;
}

/*-----------------------------------------------------------------------------
 * process_one_sample – full sections 1-11 for a single sample.
 *
 * Equivalent to FusionAhrsUpdateNoMagnetometer():
 *   halfGravity is computed from the quaternion that was normalised at the
 *   end of the PREVIOUS sample, so it is always fresh.
 *---------------------------------------------------------------------------*/
static void process_one_sample(FusionAhrs *const ahrs,
                               float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float deltaTime)
{
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

    /* Section 3 – ramp gain during initialisation */
    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if ((ahrs->rampedGain < ahrs->gain) || (ahrs->gain == 0.0f)) {
            ahrs->rampedGain          = ahrs->gain;
            ahrs->initialising        = false;
            ahrs->angularRateRecovery = false;
        }
    }

    /* Section 4 – halfGravity from the CURRENT normalised quaternion.
     * Because we normalise at the end of every sample (Sec 9), this is
     * always based on the most up-to-date orientation estimate – exactly
     * what the original FusionAhrs library does. */
    const float qw = ahrs->quaternion.element.w;
    const float qx = ahrs->quaternion.element.x;
    const float qy = ahrs->quaternion.element.y;
    const float qz = ahrs->quaternion.element.z;

    const float halfGravityX = qx * qz - qw * qy;
    const float halfGravityY = qy * qz + qw * qx;
    const float halfGravityZ = qw * qw - 0.5f + qz * qz;

    /* Sections 5-8 – HW accelerator (one sample) */
    float newQw, newQx, newQy, newQz;
    AhrsHW_Sections5to8(ahrs,
                        gx, gy, gz,
                        ax, ay, az,
                        halfGravityX, halfGravityY, halfGravityZ,
                        deltaTime,
                        &newQw, &newQx, &newQy, &newQz);

    /* Section 9 – fast-inv-sqrt normalisation */
    const float magSq = newQw*newQw + newQx*newQx + newQy*newQy + newQz*newQz;
    Union32 u = { .f = magSq };
    u.i = 0x5F1F1412 - (u.i >> 1);
    const float invMag = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);

    ahrs->quaternion.element.w = newQw * invMag;
    ahrs->quaternion.element.x = newQx * invMag;
    ahrs->quaternion.element.y = newQy * invMag;
    ahrs->quaternion.element.z = newQz * invMag;

    /* Section 10 – zero heading during initialisation
     * (mirrors FusionAhrsSetHeading(ahrs, 0.0f)) */
    if (ahrs->initialising) {
        const float yaw = atan2f(
            ahrs->quaternion.element.w * ahrs->quaternion.element.z +
            ahrs->quaternion.element.x * ahrs->quaternion.element.y,
            0.5f - ahrs->quaternion.element.y * ahrs->quaternion.element.y -
            ahrs->quaternion.element.z * ahrs->quaternion.element.z);

        const float half = 0.5f * yaw;         /* halfYawMinusHeading (heading=0) */
        const float cw   =  cosf(half);
        const float sz   = -sinf(half);         /* rotation: (cw, 0, 0, sz) */

        const float rw = ahrs->quaternion.element.w;
        const float rx = ahrs->quaternion.element.x;
        const float ry = ahrs->quaternion.element.y;
        const float rz = ahrs->quaternion.element.z;

        /* FusionQuaternionProduct(rotation, q) */
        ahrs->quaternion.element.w = cw * rw - sz * rz;
        ahrs->quaternion.element.x = cw * rx - sz * ry;
        ahrs->quaternion.element.y = cw * ry + sz * rx;
        ahrs->quaternion.element.z = cw * rz + sz * rw;
    }

    /* Section 11 – quaternion is updated in ahrs struct (caller reads it) */
}

/*-----------------------------------------------------------------------------
 * run_pipeline – iterates over every embedded sample one at a time
 *---------------------------------------------------------------------------*/
static void run_pipeline(const char *outputPath)
{
    FusionAhrs ahrs;
    ahrs_init(&ahrs);

    FILE *outFile = NULL;
    if (outputPath) {
        outFile = fopen(outputPath, "w");
        if (!outFile)
            printf("Warning: cannot open %s – output skipped.\n", outputPath);
        else
            fprintf(outFile, "time,qw,qx,qy,qz,roll,pitch,yaw\n");
    }

    const float deltaTime = 1.0f / SAMPLE_RATE;   /* 10 ms at 100 Hz */

    printf("Processing %d embedded samples one by one...\n", EMBEDDED_NUM_SAMPLES);

    for (int i = 0; i < EMBEDDED_NUM_SAMPLES; i++) {

        /* ── All data comes from compile-time constant – zero file I/O ── */
        process_one_sample(&ahrs,
                           SENSOR_DATA[i].gx, SENSOR_DATA[i].gy, SENSOR_DATA[i].gz,
                           SENSOR_DATA[i].ax, SENSOR_DATA[i].ay, SENSOR_DATA[i].az,
                           deltaTime);

        if (outFile) {
            LocalEuler e = euler_from_quat(ahrs.quaternion);
            fprintf(outFile, "%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f\n",
                    SENSOR_DATA[i].time,
                    ahrs.quaternion.element.w,
                    ahrs.quaternion.element.x,
                    ahrs.quaternion.element.y,
                    ahrs.quaternion.element.z,
                    e.angle.roll, e.angle.pitch, e.angle.yaw);
        }
    }

    if (outFile) fclose(outFile);

    LocalEuler fe = euler_from_quat(ahrs.quaternion);
    printf("Done.\n");
    if (outputPath && outFile) printf("Output: %s\n", outputPath);
    printf("\nFinal orientation:\n");
    printf("  Roll:  %.2f deg\n", fe.angle.roll);
    printf("  Pitch: %.2f deg\n", fe.angle.pitch);
    printf("  Yaw:   %.2f deg\n", fe.angle.yaw);
}

/*-----------------------------------------------------------------------------
 * main
 *---------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    const char *output_path = (argc > 1) ? argv[1] : NULL;
    run_pipeline(output_path);
    return 0;
}

