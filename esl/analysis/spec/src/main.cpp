/**
 * @file main.cpp
 * @brief AHRS Pipeline with HW/SW partitioning
 *
 * ARCHITECTURE:
 *   SW (main.cpp): Sections 1-4, 9-11, calibration, output
 *   HW (ahrs_hw.cpp): Sections 5-8 (sc_fixed, suženi tipovi)
 *
 * USAGE:
 *   make
 *   ./ahrs_pipeline sensor_data.csv orientation_data.csv
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "ahrs_hw.h"
#include <systemc.h>

//------------------------------------------------------------------------------
// Configuration
//------------------------------------------------------------------------------

#define SAMPLE_RATE 100
#define MAX_SAMPLES 100000

//------------------------------------------------------------------------------
// Structures
//------------------------------------------------------------------------------

typedef struct {
    float time;
    float gx, gy, gz;
    float ax, ay, az;
} SensorSample;

typedef struct {
    FusionQuaternion quaternion;
} AhrsOutput;

// Union for Fast Inverse Sqrt
typedef union {
    float f;
    int32_t i;
} Union32;

//------------------------------------------------------------------------------
// Initialization
//------------------------------------------------------------------------------

void FusionOffsetInitialise(FusionOffset *const offset, const unsigned int sampleRate) {
    offset->filterCoefficient = 0.01f;
    offset->timeout = 5 * sampleRate;
    offset->timer = 0;
    offset->gyroscopeOffset = FUSION_VECTOR_ZERO;
}

void FusionAhrsInitialise(FusionAhrs *const ahrs) {
    memset(ahrs, 0, sizeof(FusionAhrs));
    ahrs->quaternion = FUSION_IDENTITY_QUATERNION;
    ahrs->initialising = true;
    ahrs->rampedGain = 10.0f;
    ahrs->gain = 0.5f;
    ahrs->gyroscopeRange = 2000.0f;
    // Ispravka 1: precomputed threshold (0.5 * sin(10°))^2, ne hardkodirano 10.0
    ahrs->accelerationRejection = powf(0.5f * sinf(10.0f * (float)M_PI / 180.0f), 2);
    ahrs->recoveryTriggerPeriod = 500;
    ahrs->rampedGainStep = (10.0f - 0.5f) / 3.0f;
    ahrs->accelerationRecoveryTimeout = 500;
}

//------------------------------------------------------------------------------
// SW Functions - Calibration
//------------------------------------------------------------------------------

static inline FusionVector FusionCalibrationInertial(
    const FusionVector uncalibrated,
    const FusionMatrix misalignment,
    const FusionVector sensitivity,
    const FusionVector offset
) {
    FusionVector result;
    float ux = uncalibrated.axis.x - offset.axis.x;
    float uy = uncalibrated.axis.y - offset.axis.y;
    float uz = uncalibrated.axis.z - offset.axis.z;

    result.axis.x = (misalignment.array[0][0]*ux + misalignment.array[0][1]*uy + misalignment.array[0][2]*uz) * sensitivity.axis.x;
    result.axis.y = (misalignment.array[1][0]*ux + misalignment.array[1][1]*uy + misalignment.array[1][2]*uz) * sensitivity.axis.y;
    result.axis.z = (misalignment.array[2][0]*ux + misalignment.array[2][1]*uy + misalignment.array[2][2]*uz) * sensitivity.axis.z;

    return result;
}

FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope) {
    offset->timer++;

    if (fabsf(gyroscope.axis.x) < 3.0f &&
        fabsf(gyroscope.axis.y) < 3.0f &&
        fabsf(gyroscope.axis.z) < 3.0f) {
        if (offset->timer >= offset->timeout) {
            offset->gyroscopeOffset.axis.x += gyroscope.axis.x * offset->filterCoefficient;
            offset->gyroscopeOffset.axis.y += gyroscope.axis.y * offset->filterCoefficient;
            offset->gyroscopeOffset.axis.z += gyroscope.axis.z * offset->filterCoefficient;
        }
    } else {
        offset->timer = 0;
    }

    gyroscope.axis.x -= offset->gyroscopeOffset.axis.x;
    gyroscope.axis.y -= offset->gyroscopeOffset.axis.y;
    gyroscope.axis.z -= offset->gyroscopeOffset.axis.z;

    return gyroscope;
}

//------------------------------------------------------------------------------
// SW Function - Euler Conversion
//------------------------------------------------------------------------------

FusionEuler FusionEulerFrom(const FusionQuaternion q) {
    FusionEuler euler;

    euler.angle.roll = atan2f(
        q.element.w * q.element.x + q.element.y * q.element.z,
        0.5f - q.element.y * q.element.y - q.element.x * q.element.x
    ) * (180.0f / M_PI);

    euler.angle.pitch = asinf(
        2.0f * (q.element.w * q.element.y - q.element.z * q.element.x)
    ) * (180.0f / M_PI);

    euler.angle.yaw = atan2f(
        q.element.w * q.element.z + q.element.x * q.element.y,
        0.5f - q.element.y * q.element.y - q.element.z * q.element.z
    ) * (180.0f / M_PI);

    return euler;
}

//------------------------------------------------------------------------------
// AHRS UPDATE - single sample: SW 1-4, HW 5-8, SW 9-11
//------------------------------------------------------------------------------

void FusionAhrsUpdate(
    FusionAhrs *const ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float deltaTime
) {
    // SECTION 1
    ahrs->accelerometer.axis.x = ax;
    ahrs->accelerometer.axis.y = ay;
    ahrs->accelerometer.axis.z = az;

    // SECTION 2 - Ispravka 2: puno resetovanje stanja pri prekoračenju gyro range-a
    if ((fabsf(gx) > ahrs->gyroscopeRange) ||
        (fabsf(gy) > ahrs->gyroscopeRange) ||
        (fabsf(gz) > ahrs->gyroscopeRange)) {
        const FusionQuaternion savedQuat = ahrs->quaternion;
        ahrs->rampedGain = 10.0f;
        ahrs->initialising = true;
        ahrs->halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
        ahrs->accelerometerIgnored = false;
        ahrs->accelerationRecoveryTrigger = 0;
        ahrs->accelerationRecoveryTimeout = (int)ahrs->recoveryTriggerPeriod;
        ahrs->quaternion = savedQuat;
        ahrs->angularRateRecovery = true;
    }

    // SECTION 3
    if (ahrs->initialising) {
        ahrs->rampedGain -= ahrs->rampedGainStep * deltaTime;
        if ((ahrs->rampedGain < ahrs->gain) || (ahrs->gain == 0.0f)) {
            ahrs->rampedGain = ahrs->gain;
            ahrs->initialising = false;
            ahrs->angularRateRecovery = false;
        }
    }

    // SECTION 4
    const float qw = ahrs->quaternion.element.w;
    const float qx = ahrs->quaternion.element.x;
    const float qy = ahrs->quaternion.element.y;
    const float qz = ahrs->quaternion.element.z;
    const float halfGravityX = qx * qz - qw * qy;
    const float halfGravityY = qy * qz + qw * qx;
    const float halfGravityZ = qw * qw - 0.5f + qz * qz;

    // SECTIONS 5-8: HW (sc_fixed implementacija)
    float newQw, newQx, newQy, newQz;
    AhrsHW_Sections5to8(ahrs, gx, gy, gz, ax, ay, az,
                         halfGravityX, halfGravityY, halfGravityZ,
                         deltaTime, &newQw, &newQx, &newQy, &newQz);

    // SECTION 9: NORMALIZE
    const float mag2 = newQw*newQw + newQx*newQx + newQy*newQy + newQz*newQz;
    Union32 uq = {.f = mag2};
    uq.i = 0x5F1F1412 - (uq.i >> 1);
    const float invMag = uq.f * (1.69000231f - 0.714158168f * mag2 * uq.f * uq.f);
    ahrs->quaternion.element.w = newQw * invMag;
    ahrs->quaternion.element.x = newQx * invMag;
    ahrs->quaternion.element.y = newQy * invMag;
    ahrs->quaternion.element.z = newQz * invMag;

    // SECTION 10: ZERO HEADING (samo tokom inicijalizacije)
    if (ahrs->initialising) {
        const float yaw = atan2f(
            ahrs->quaternion.element.w * ahrs->quaternion.element.z +
            ahrs->quaternion.element.x * ahrs->quaternion.element.y,
            0.5f - ahrs->quaternion.element.y * ahrs->quaternion.element.y -
            ahrs->quaternion.element.z * ahrs->quaternion.element.z);
        const float halfYaw = 0.5f * yaw;
        const float cosHY = cosf(halfYaw), sinHY = sinf(halfYaw);
        const float rqw = ahrs->quaternion.element.w;
        const float rqx = ahrs->quaternion.element.x;
        const float rqy = ahrs->quaternion.element.y;
        const float rqz = ahrs->quaternion.element.z;
        ahrs->quaternion.element.w =  cosHY * rqw + sinHY * rqz;
        ahrs->quaternion.element.x =  cosHY * rqx + sinHY * rqy;
        ahrs->quaternion.element.y =  cosHY * rqy - sinHY * rqx;
        ahrs->quaternion.element.z =  cosHY * rqz - sinHY * rqw;
    }
}

//------------------------------------------------------------------------------
// Data Loading
//------------------------------------------------------------------------------

int load_binary_data(const char* filename, SensorSample* samples, int max_samples) {
    FILE* file = fopen(filename, "rb");
    if (!file) return -1;

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    int num_samples = file_size / sizeof(SensorSample);
    if (num_samples > max_samples) num_samples = max_samples;

    size_t read = fread(samples, sizeof(SensorSample), num_samples, file);
    fclose(file);

    return (int)read;
}

int load_csv_data(const char* filename, SensorSample* samples, int max_samples) {
    FILE* file = fopen(filename, "r");
    if (!file) return -1;

    char line[256];
    fgets(line, sizeof(line), file);  // skip header

    int count = 0;
    while (fgets(line, sizeof(line), file) && count < max_samples) {
        float time, gx, gy, gz, ax, ay, az;
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f", &time, &gx, &gy, &gz, &ax, &ay, &az) == 7) {
            samples[count].time  = time;
            samples[count].gx = gx; samples[count].gy = gy; samples[count].gz = gz;
            samples[count].ax = ax; samples[count].ay = ay; samples[count].az = az;
            count++;
        }
    }

    fclose(file);
    return count;
}

//------------------------------------------------------------------------------
// Pipeline - sample-by-sample
//------------------------------------------------------------------------------

void run_pipeline(SensorSample* rawSamples, int numSamples, const char* outputPath) {

    const FusionMatrix gyroMisalign  = FUSION_IDENTITY_MATRIX;
    const FusionVector gyroSens      = FUSION_VECTOR_ONES;
    const FusionVector gyroOffset    = FUSION_VECTOR_ZERO;
    const FusionMatrix accelMisalign = FUSION_IDENTITY_MATRIX;
    const FusionVector accelSens     = FUSION_VECTOR_ONES;
    const FusionVector accelOffset   = FUSION_VECTOR_ZERO;

    FusionOffset offset;
    FusionOffsetInitialise(&offset, SAMPLE_RATE);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    FILE* outFile = fopen(outputPath, "w");
    if (!outFile) {
        printf("Error: Cannot open %s\n", outputPath);
        return;
    }
    fprintf(outFile, "time,qw,qx,qy,qz,roll,pitch,yaw\n");

    const float deltaTime = 1.0f / SAMPLE_RATE;

    printf("Processing %d samples...\n", numSamples);

    for (int i = 0; i < numSamples; i++) {

        FusionVector gyro  = {.axis = { .x = rawSamples[i].gx, .y = rawSamples[i].gy, .z = rawSamples[i].gz }};
        FusionVector accel = {.axis = { .x = rawSamples[i].ax, .y = rawSamples[i].ay, .z = rawSamples[i].az }};

        gyro  = FusionCalibrationInertial(gyro,  gyroMisalign,  gyroSens,  gyroOffset);
        accel = FusionCalibrationInertial(accel, accelMisalign, accelSens, accelOffset);
        gyro  = FusionOffsetUpdate(&offset, gyro);

        FusionAhrsUpdate(&ahrs, gyro.axis.x, gyro.axis.y, gyro.axis.z,
                          accel.axis.x, accel.axis.y, accel.axis.z, deltaTime);

        FusionEuler euler = FusionEulerFrom(ahrs.quaternion);
        fprintf(outFile, "%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f\n",
                rawSamples[i].time,
                ahrs.quaternion.element.w, ahrs.quaternion.element.x,
                ahrs.quaternion.element.y, ahrs.quaternion.element.z,
                euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    }

    fclose(outFile);

    FusionEuler final_euler = FusionEulerFrom(ahrs.quaternion);
    printf("\nDone: %d samples processed.\n", numSamples);
    printf("Output: %s\n\n", outputPath);
    printf("Final orientation:\n");
    printf("  Roll:  %.2f deg\n", final_euler.angle.roll);
    printf("  Pitch: %.2f deg\n", final_euler.angle.pitch);
    printf("  Yaw:   %.2f deg\n", final_euler.angle.yaw);
}

//------------------------------------------------------------------------------
// Main (sc_main jer koristimo SystemC za sc_fixed u ahrs_hw.cpp)
//------------------------------------------------------------------------------

int sc_main(int argc, char* argv[]) {

    SensorSample* samples = (SensorSample*)malloc(MAX_SAMPLES * sizeof(SensorSample));
    if (!samples) {
        printf("Error: Cannot allocate memory.\n");
        return 1;
    }

    const char* input_path  = "test_10_samples.csv";
    const char* output_path = "orientation_data.csv";

    if (argc > 1) input_path  = argv[1];
    if (argc > 2) output_path = argv[2];

    const char* ext = strrchr(input_path, '.');
    int num_samples;
    if (ext && strcmp(ext, ".bin") == 0) {
        num_samples = load_binary_data(input_path, samples, MAX_SAMPLES);
        if (num_samples < 0) {
            printf("Error: Cannot load %s\n", input_path);
            free(samples);
            return 1;
        }
        printf("Loaded %d samples from %s (binary)\n\n", num_samples, input_path);
    } else {
        num_samples = load_csv_data(input_path, samples, MAX_SAMPLES);
        if (num_samples < 0) {
            printf("Error: Cannot load %s\n", input_path);
            free(samples);
            return 1;
        }
        printf("Loaded %d samples from %s (CSV)\n\n", num_samples, input_path);
    }

    run_pipeline(samples, num_samples, output_path);

    free(samples);
    return 0;
}
