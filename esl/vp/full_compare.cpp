/**
 * @file full_compare.cpp
 * @brief Sveobuhvatno poređenje: C++ Spec vs VP algoritam
 *
 * Čita sensor_data_short.csv (498 uzoraka realnih podataka senzora) i
 * paralelno pokreće dva algoritma:
 *
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │ SPEC  algoritam (ahrs_hw.cpp + main.cpp)                            │
 *  │   SW sekcije 1-4 : float   (kao u spec main.cpp)                   │
 *  │   HW sekcije 5-8 : sc_fixed (kao u spec ahrs_hw.cpp V3 Final)      │
 *  │   SW sekcije 9-11: float   (kao u spec SW_ProcessOutput)            │
 *  ├─────────────────────────────────────────────────────────────────────┤
 *  │ VP  algoritam (ahrs_ip.cpp + cpu.cpp)                               │
 *  │   SW sekcije 1-4 : float   (kao u VP cpu.cpp sw_prepare_sample)    │
 *  │   HW sekcije 5-8 : sc_fixed (kao u VP ahrs_ip.cpp hw_sections5to8) │
 *  │   SW sekcije 9-11: float   (kao u VP cpu.cpp sw_process_output)    │
 *  └─────────────────────────────────────────────────────────────────────┘
 *
 * Prikazuje kvaternion po uzorku za oba algoritma i razliku (maxΔ).
 *
 * Build: g++ -std=c++17 -DSC_INCLUDE_FX -I/usr/local/include -I. \
 *            -o full_compare full_compare.cpp \
 *            -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lsystemc -lm
 * Run:   ./full_compare
 */

#include <systemc>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>

using namespace sc_dt;

// ============================================================================
// Putanja do CSV fajla sa podacima senzora
// ============================================================================
static const char *CSV_PATH =
    "../analysis/data/sensor_data_full.csv";

// Maksimalan broj uzoraka koje ćemo obraditi
static const int MAX_SAMPLES = 2000;

// ============================================================================
// sc_fixed tipovi – identični sa defines.hpp i ahrs_hw.cpp V3 Final
// ============================================================================
typedef sc_fixed<29, 2, SC_RND, SC_SAT>  quat_t;
typedef sc_fixed<26, 1, SC_RND, SC_SAT>  dot_t;
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  norm_t;
typedef sc_fixed<20, 3, SC_RND, SC_SAT>  accel_t;
typedef sc_fixed<26, 2, SC_RND, SC_SAT>  hg_t;
typedef sc_ufixed<22, 2, SC_RND, SC_SAT> magsq_t;
typedef sc_ufixed<24, 6, SC_RND, SC_SAT> invmag_t;
typedef sc_fixed<22, 2, SC_RND, SC_SAT>  halfgyro_t;
typedef sc_ufixed<20, 4, SC_RND, SC_SAT> gain_t;
typedef sc_fixed<26, 5, SC_RND, SC_SAT>  adjhg_t;
typedef sc_fixed<27, 5, SC_RND, SC_SAT>  dq_t;
typedef sc_ufixed<20, 0, SC_RND, SC_SAT> dt_t;
typedef sc_fixed<22, 12, SC_RND, SC_SAT> gyro_raw_t;

// ============================================================================
// Konstante AHRS
// ============================================================================
static const float GAIN            = 0.5f;
static const float GYRO_RANGE_F    = 2000.0f;
static const float DT_F            = 0.01f;     // 100 Hz
static const float DEG_TO_HALF_RAD = 3.14159265358979f / 360.0f;

// ============================================================================
// Fast inverse square root (identično spec-u i VP-u)
// ============================================================================
static inline float fast_inv_sqrt(float x)
{
    union { float f; int32_t i; } u = {x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

// ============================================================================
// Čitanje CSV fajla
// ============================================================================
struct SensorRow { float gx, gy, gz, ax, ay, az; };

static int load_csv(const char *path, SensorRow *rows, int max_rows)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "Ne mogu da otvorim: %s\n", path); return -1; }
    char line[512];
    fgets(line, sizeof(line), f);  // preskoči zaglavlje
    int n = 0;
    while (fgets(line, sizeof(line), f) && n < max_rows) {
        float t;
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f",
                   &t,
                   &rows[n].gx, &rows[n].gy, &rows[n].gz,
                   &rows[n].ax, &rows[n].ay, &rows[n].az) == 7) {
            n++;
        }
    }
    fclose(f);
    return n;
}

// ============================================================================
//                         SPEC ALGORITAM
//        SW sekcije 1-4 i 9-11: float   (kao u main.cpp)
//        HW sekcije 5-8:         sc_fixed (kao u ahrs_hw.cpp V3 Final)
// ============================================================================
struct SpecState {
    float qw = 1.f, qx = 0.f, qy = 0.f, qz = 0.f;  // normalizovani kvaternion
    float hw_qw = 1.f, hw_qx = 0.f, hw_qy = 0.f, hw_qz = 0.f; // HW interno stanje
    float rampedGain = 10.f;
    float rampedGainStep;
    bool  initialising = true;

    SpecState() { rampedGainStep = (10.f - GAIN) / 3.f; }
};

// HW sekcije 5-8 – sc_fixed interno (identično ahrs_hw.cpp AhrsHW_Sections5to8)
static void spec_hw_5to8(
    float hw_qw, float hw_qx, float hw_qy, float hw_qz,
    float gx_f,  float gy_f,  float gz_f,
    float ax_f,  float ay_f,  float az_f,
    float hgX_f, float hgY_f, float hgZ_f,
    float rampedGain_f,
    float &out_qw, float &out_qx, float &out_qy, float &out_qz)
{
    // Učitaj kvaternion u sc_fixed
    quat_t qw = hw_qw, qx = hw_qx, qy = hw_qy, qz = hw_qz;

    // Ulazni podaci u sc_fixed
    const accel_t ax_fp = ax_f, ay_fp = ay_f, az_fp = az_f;
    const hg_t    hgX   = hgX_f, hgY = hgY_f, hgZ = hgZ_f;
    const dt_t    dt    = DT_F;
    const gain_t  gain  = rampedGain_f;

    // Sekcija 6: žiroskop deg/s → polu-ugao rad/s
    const halfgyro_t halfGyroX = gx_f * DEG_TO_HALF_RAD;
    const halfgyro_t halfGyroY = gy_f * DEG_TO_HALF_RAD;
    const halfgyro_t halfGyroZ = gz_f * DEG_TO_HALF_RAD;

    // Sekcija 5: povratna sprega akcelerometra
    norm_t fbX = 0.f, fbY = 0.f, fbZ = 0.f;
    const bool accelNotZero = !((ax_f == 0.f) && (ay_f == 0.f) && (az_f == 0.f));
    if (accelNotZero) {
        const magsq_t  accelMagSq  = ax_fp * ax_fp + ay_fp * ay_fp + az_fp * az_fp;
        const invmag_t accelInvMag = fast_inv_sqrt((float)accelMagSq);
        const norm_t   nax = ax_fp * accelInvMag;
        const norm_t   nay = ay_fp * accelInvMag;
        const norm_t   naz = az_fp * accelInvMag;

        norm_t crossX = nay * hgZ - naz * hgY;
        norm_t crossY = naz * hgX - nax * hgZ;
        norm_t crossZ = nax * hgY - nay * hgX;

        const dot_t dot = nax * hgX + nay * hgY + naz * hgZ;
        if ((float)dot < 0.f) {
            const magsq_t  crossMagSq  = crossX * crossX + crossY * crossY + crossZ * crossZ;
            const invmag_t crossInvMag = fast_inv_sqrt((float)crossMagSq);
            crossX = crossX * crossInvMag;
            crossY = crossY * crossInvMag;
            crossZ = crossZ * crossInvMag;
        }
        fbX = crossX;  fbY = crossY;  fbZ = crossZ;
    }

    // Sekcija 7: primeni gain
    const adjhg_t adjX = halfGyroX + fbX * gain;
    const adjhg_t adjY = halfGyroY + fbY * gain;
    const adjhg_t adjZ = halfGyroZ + fbZ * gain;

    // Sekcija 8: integracija kvaterniona
    const dq_t dqw = -qx * adjX - qy * adjY - qz * adjZ;
    const dq_t dqx =  qw * adjX + qy * adjZ - qz * adjY;
    const dq_t dqy =  qw * adjY - qx * adjZ + qz * adjX;
    const dq_t dqz =  qw * adjZ + qx * adjY - qy * adjX;

    const quat_t rqw = qw + dqw * dt;
    const quat_t rqx = qx + dqx * dt;
    const quat_t rqy = qy + dqy * dt;
    const quat_t rqz = qz + dqz * dt;

    out_qw = (float)rqw;
    out_qx = (float)rqx;
    out_qy = (float)rqy;
    out_qz = (float)rqz;
}

static void spec_step(SpecState &s, float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float &qw_out, float &qx_out, float &qy_out, float &qz_out)
{
    // SW sekcija 1: čuvaj akcelerometar (samo prenos)
    // SW sekcija 2: provera opsega žiroskopa
    if (fabsf(gx) > GYRO_RANGE_F || fabsf(gy) > GYRO_RANGE_F || fabsf(gz) > GYRO_RANGE_F) {
        /* angularRateRecovery = true */
    }
    // SW sekcija 3: ramp gain
    if (s.initialising) {
        s.rampedGain -= s.rampedGainStep * DT_F;
        if (s.rampedGain < GAIN || GAIN == 0.f) {
            s.rampedGain = GAIN;
            s.initialising = false;
        }
    }
    // SW sekcija 4: halfGravity od trenutnog normalizovanog kvaterniona
    const float hgX = s.qx * s.qz - s.qw * s.qy;
    const float hgY = s.qy * s.qz + s.qw * s.qx;
    const float hgZ = s.qw * s.qw - 0.5f + s.qz * s.qz;

    // HW sekcije 5-8 (sc_fixed interno)
    float uqw, uqx, uqy, uqz;
    spec_hw_5to8(s.hw_qw, s.hw_qx, s.hw_qy, s.hw_qz,
                 gx, gy, gz, ax, ay, az,
                 hgX, hgY, hgZ, s.rampedGain,
                 uqw, uqx, uqy, uqz);
    s.hw_qw = uqw;  s.hw_qx = uqx;
    s.hw_qy = uqy;  s.hw_qz = uqz;

    // SW sekcija 9: normalizacija kvaterniona
    const float magSq = uqw*uqw + uqx*uqx + uqy*uqy + uqz*uqz;
    union { float f; int32_t i; } u;
    u.f = magSq;
    u.i = 0x5F1F1412 - (u.i >> 1);
    const float inv = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);
    s.qw = uqw * inv;  s.qx = uqx * inv;
    s.qy = uqy * inv;  s.qz = uqz * inv;
    // Sinhronizuj HW interono stanje sa normalizovanim
    s.hw_qw = s.qw;  s.hw_qx = s.qx;
    s.hw_qy = s.qy;  s.hw_qz = s.qz;

    // SW sekcija 10: korekcija nultog kursa (samo tokom inicijalizacije)
    if (s.initialising) {
        const float yaw = atan2f(s.qw * s.qz + s.qx * s.qy,
                                 0.5f - s.qy * s.qy - s.qz * s.qz);
        const float hy = 0.5f * yaw;
        const float cy = cosf(hy), sy = sinf(hy);
        const float nw =  cy * s.qw + sy * s.qz;
        const float nx =  cy * s.qx + sy * s.qy;
        const float ny =  cy * s.qy - sy * s.qx;
        const float nz =  cy * s.qz - sy * s.qw;
        s.qw = nw;  s.qx = nx;  s.qy = ny;  s.qz = nz;
        s.hw_qw = nw;  s.hw_qx = nx;  s.hw_qy = ny;  s.hw_qz = nz;
    }

    qw_out = s.qw;  qx_out = s.qx;
    qy_out = s.qy;  qz_out = s.qz;
}

// ============================================================================
//                          VP ALGORITAM
//        SW sekcije 1-4 i 9-11: sc_fixed  (kao u cpu.cpp)
//        HW sekcije 5-8:         sc_fixed  (kao u ahrs_ip.cpp)
// ============================================================================
struct VpState {
    // SW state: float (Sekcije 1-4, 9-11) – identično cpu.hpp nakon refaktorisanja
    float qw = 1.f, qx = 0.f, qy = 0.f, qz = 0.f;
    // HW interno stanje: sc_fixed quat_t (Sekcije 5-8)
    quat_t hw_qw = 1.f, hw_qx = 0.f, hw_qy = 0.f, hw_qz = 0.f;
    // Gain varijable: float (SW)
    float rampedGain;
    float rampedGainStep;
    float gain           = GAIN;
    // Delta vreme: float (SW)
    float deltaTime      = DT_F;
    // Žiroskopski opseg: float (SW)
    float gyroscopeRange = GYRO_RANGE_F;
    // Zastavice: bool (SW)
    bool initialising        = true;
    bool angularRateRecovery = false;

    VpState() {
        rampedGain     = 10.f;
        rampedGainStep = (10.f - GAIN) / 3.f;
    }
};

// HW sekcije 5-8 – potpuno sc_fixed (identično ahrs_ip.cpp hw_sections5to8)
static void vp_hw_5to8(VpState &s,
                       gyro_raw_t gx_fp, gyro_raw_t gy_fp, gyro_raw_t gz_fp,
                       accel_t ax_fp, accel_t ay_fp, accel_t az_fp,
                       hg_t hgX, hg_t hgY, hg_t hgZ,
                       gain_t rampedGain,
                       quat_t &out_qw, quat_t &out_qx,
                       quat_t &out_qy, quat_t &out_qz)
{
    // Sekcija 6: žiroskop → polu-ugao rad/s
    const halfgyro_t halfGyroX = gx_fp * DEG_TO_HALF_RAD;
    const halfgyro_t halfGyroY = gy_fp * DEG_TO_HALF_RAD;
    const halfgyro_t halfGyroZ = gz_fp * DEG_TO_HALF_RAD;

    // Sekcija 5: povratna sprega akcelerometra
    norm_t fbX = 0.f, fbY = 0.f, fbZ = 0.f;
    const bool accelNotZero = !((float)ax_fp == 0.f && (float)ay_fp == 0.f && (float)az_fp == 0.f);
    if (accelNotZero) {
        const magsq_t  accelMagSq  = ax_fp * ax_fp + ay_fp * ay_fp + az_fp * az_fp;
        const invmag_t accelInvMag = fast_inv_sqrt((float)accelMagSq);
        const norm_t   nax = ax_fp * accelInvMag;
        const norm_t   nay = ay_fp * accelInvMag;
        const norm_t   naz = az_fp * accelInvMag;

        norm_t crossX = nay * hgZ - naz * hgY;
        norm_t crossY = naz * hgX - nax * hgZ;
        norm_t crossZ = nax * hgY - nay * hgX;

        const dot_t dot = nax * hgX + nay * hgY + naz * hgZ;
        if ((float)dot < 0.f) {
            const magsq_t  crossMagSq  = crossX * crossX + crossY * crossY + crossZ * crossZ;
            const invmag_t crossInvMag = fast_inv_sqrt((float)crossMagSq);
            crossX = crossX * crossInvMag;
            crossY = crossY * crossInvMag;
            crossZ = crossZ * crossInvMag;
        }
        fbX = crossX;  fbY = crossY;  fbZ = crossZ;
    }

    // Sekcija 7: primeni gain
    const adjhg_t adjX = halfGyroX + fbX * rampedGain;
    const adjhg_t adjY = halfGyroY + fbY * rampedGain;
    const adjhg_t adjZ = halfGyroZ + fbZ * rampedGain;

    // Sekcija 8: integracija kvaterniona
    const dq_t dqw = -s.hw_qx * adjX - s.hw_qy * adjY - s.hw_qz * adjZ;
    const dq_t dqx =  s.hw_qw * adjX + s.hw_qy * adjZ - s.hw_qz * adjY;
    const dq_t dqy =  s.hw_qw * adjY - s.hw_qx * adjZ + s.hw_qz * adjX;
    const dq_t dqz =  s.hw_qw * adjZ + s.hw_qx * adjY - s.hw_qy * adjX;

    const dt_t dt_fixed = s.deltaTime;  // float -> dt_t za HW preciznost
    out_qw = s.hw_qw + dqw * dt_fixed;
    out_qx = s.hw_qx + dqx * dt_fixed;
    out_qy = s.hw_qy + dqy * dt_fixed;
    out_qz = s.hw_qz + dqz * dt_fixed;

    s.hw_qw = out_qw;  s.hw_qx = out_qx;
    s.hw_qy = out_qy;  s.hw_qz = out_qz;
}

static void vp_step(VpState &s, float gx_f, float gy_f, float gz_f,
                    float ax_f, float ay_f, float az_f,
                    float &qw_out, float &qx_out, float &qy_out, float &qz_out)
{
    // Kvantizuj ulaze senzora u sc_fixed (isti tip kao u cpu.cpp RawSensorSample)
    const gyro_raw_t gx = gx_f, gy = gy_f, gz = gz_f;
    const accel_t    ax = ax_f, ay = ay_f, az = az_f;

    // SW sekcija 1: prenos akcelerometra
    // SW sekcija 2: provera opsega žiroskopa (float, identično cpu.cpp)
    if (fabsf((float)gx) > s.gyroscopeRange ||
        fabsf((float)gy) > s.gyroscopeRange ||
        fabsf((float)gz) > s.gyroscopeRange) {
        s.angularRateRecovery = true;
    }
    // SW sekcija 3: ramp gain (float – identično cpu.cpp)
    if (s.initialising) {
        s.rampedGain -= s.rampedGainStep * s.deltaTime;
        if (s.rampedGain < s.gain || s.gain == 0.f) {
            s.rampedGain = s.gain;
            s.initialising = false;
        }
    }
    // SW sekcija 4: halfGravity od float kvaterniona → kvantizuj na sc_fixed za HW
    const hg_t hgX = s.qx * s.qz - s.qw * s.qy;
    const hg_t hgY = s.qy * s.qz + s.qw * s.qx;
    const hg_t hgZ = s.qw * s.qw - 0.5f + s.qz * s.qz;

    // HW sekcije 5-8 (potpuno sc_fixed)
    quat_t uqw, uqx, uqy, uqz;
    vp_hw_5to8(s, gx, gy, gz, ax, ay, az,
               hgX, hgY, hgZ, (gain_t)s.rampedGain,
               uqw, uqx, uqy, uqz);

    // SW sekcija 9: normalizacija – čita sc_fixed iz HW, računa u float
    const float nqw = (float)uqw, nqx = (float)uqx;
    const float nqy = (float)uqy, nqz = (float)uqz;
    const float magSq = nqw*nqw + nqx*nqx + nqy*nqy + nqz*nqz;
    union { float f; int32_t i; } u;
    u.f = magSq;
    u.i = 0x5F1F1412 - (u.i >> 1);
    const float inv = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);
    s.qw = nqw * inv;  s.qx = nqx * inv;
    s.qy = nqy * inv;  s.qz = nqz * inv;
    // Sinhronizuj HW sc_fixed stanje sa normalizovanim float stanjem
    s.hw_qw = s.qw;  s.hw_qx = s.qx;
    s.hw_qy = s.qy;  s.hw_qz = s.qz;

    // SW sekcija 10: korekcija nultog kursa (float)
    if (s.initialising) {
        const float yaw = atan2f(s.qw * s.qz + s.qx * s.qy,
                                 0.5f - s.qy * s.qy - s.qz * s.qz);
        const float hy = 0.5f * yaw;
        const float cy = cosf(hy), sy = sinf(hy);
        const float nw =  cy * s.qw + sy * s.qz;
        const float nx =  cy * s.qx + sy * s.qy;
        const float ny =  cy * s.qy - sy * s.qx;
        const float nz =  cy * s.qz - sy * s.qw;
        s.qw = nw;  s.qx = nx;  s.qy = ny;  s.qz = nz;
        s.hw_qw = nw;  s.hw_qx = nx;  s.hw_qy = ny;  s.hw_qz = nz;
    }

    qw_out = s.qw;  qx_out = s.qx;
    qy_out = s.qy;  qz_out = s.qz;
}

// ============================================================================
// sc_main
// ============================================================================
int sc_main(int /*argc*/, char * /*argv*/[])
{
    // Učitaj CSV podatke
    static SensorRow rows[MAX_SAMPLES];
    int n = load_csv(CSV_PATH, rows, MAX_SAMPLES);
    if (n < 0) return 1;

    printf("\n");
    printf("=========================================================================\n");
    printf("  POREĐENJE: C++ Spec vs VP Algoritam\n");
    printf("  Ulaz: %s  (%d uzoraka)\n", CSV_PATH, n);
    printf("-------------------------------------------------------------------------\n");
    printf("  SPEC: SW sekcije 1-4 i 9-11 float  + HW sekcije 5-8 sc_fixed\n");
    printf("  VP:   SW sekcije 1-4 i 9-11 float  + HW sekcije 5-8 sc_fixed\n");
    printf("=========================================================================\n\n");

    printf("  %4s | %-48s | %-48s | %9s\n",
           "Uzr", "         SPEC  (qw, qx, qy, qz)",
           "          VP   (qw, qx, qy, qz)", "  maxΔ");
    printf("  -----+%-48s+%-48s+----------\n",
           "------------------------------------------------",
           "------------------------------------------------");

    SpecState spec;
    VpState   vp;

    float max_overall = 0.f;
    int   mismatches  = 0;
    const float TOL = 1e-4f;

    for (int i = 0; i < n; i++) {
        float sq[4], vq[4];

        spec_step(spec, rows[i].gx, rows[i].gy, rows[i].gz,
                        rows[i].ax, rows[i].ay, rows[i].az,
                        sq[0], sq[1], sq[2], sq[3]);

        vp_step(vp, rows[i].gx, rows[i].gy, rows[i].gz,
                    rows[i].ax, rows[i].ay, rows[i].az,
                    vq[0], vq[1], vq[2], vq[3]);

        float maxd = 0.f;
        for (int k = 0; k < 4; k++)
            maxd = fmaxf(maxd, fabsf(sq[k] - vq[k]));

        if (maxd > max_overall) max_overall = maxd;
        const bool ok = maxd < TOL;
        if (!ok) mismatches++;

        // Ispiši svaki uzorak
        printf("  %4d | %+9.6f %+9.6f %+9.6f %+9.6f | %+9.6f %+9.6f %+9.6f %+9.6f | %.2e %s\n",
               i + 1,
               sq[0], sq[1], sq[2], sq[3],
               vq[0], vq[1], vq[2], vq[3],
               maxd, ok ? "✓" : "✗");
    }

    printf("\n=========================================================================\n");
    printf("  Finalni kvaternion (Spec): [%+.6f, %+.6f, %+.6f, %+.6f]\n",
           spec.qw, spec.qx, spec.qy, spec.qz);
    printf("  Finalni kvaternion (VP):   [%+.6f, %+.6f, %+.6f, %+.6f]\n",
           vp.qw, vp.qx, vp.qy, vp.qz);
    printf("-------------------------------------------------------------------------\n");
    printf("  Maksimalna greška (maxΔ): %.4e\n", max_overall);
    printf("  Neusaglašeni uzorci (>%.0e): %d / %d\n", TOL, mismatches, n);
    printf("  Ukupno: %s\n",
           (mismatches == 0) ? "SVI UZORCI ODGOVARAJU ✓" : "POSTOJE RAZLIKE ✗");
    printf("=========================================================================\n\n");

    return (mismatches == 0) ? 0 : 1;
}
