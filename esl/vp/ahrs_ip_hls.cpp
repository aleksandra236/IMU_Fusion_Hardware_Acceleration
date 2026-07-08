/**
 * @file ahrs_ip_hls.cpp
 * @brief AHRS IP core – Vitis HLS verzija (sekcije 5-8), 1 uzorak po pozivu
 *
 * Jedan poziv = jedan uzorak. Nema petlje, nema nizova.
 * HLS sintetiše čistu kombinacionu/pipelined putanju bez loop overhead-a.
 *
 * Tipovi usklađeni sa analysis/spec/src/ahrs_hw.cpp (V3 Final):
 *   quat_t : 18-bit  →  quat(18) × adjhg(26) = 44-bit ≤ 45 → 1 DSP48E1 ✓
 *   dt_t   : 20-bit  (18-bit daje grešku ~30° zbog dt=0.01 kvantizacije)
 *   dq_t   : 27-bit  →  dq(27) × dt(20) = 47-bit → 2-DSP kaskada (prihvatljivo)
 *
 * Build u Vitis HLS:
 *   - Part:   xc7z010clg400-1  (Zybo Z7-10)
 *   - Clock:  10 ns (100 MHz)
 *   - Top:    ahrs_ip_top
 */

#include <ap_fixed.h>
#include <hls_math.h>
#include <stdint.h>

// ============================================================================
// ap_fixed tipovi – identični sa analysis/spec/src/ahrs_hw.cpp (V3 Final)
//
// DSP48E1 (xc7z010): max 18-bit(B) × 27-bit(A) = 45-bit
//   quat(18) × adjhg(26) = 44-bit → 1 DSP ✓
//   dq(27)   × dt(20)    = 47-bit → 2-DSP kaskada (neizbježno)
// ============================================================================
typedef ap_fixed <18, 2, AP_RND, AP_SAT> quat_t;      // kvaternion     [-2, 2),  16 frac bita
typedef ap_fixed <26, 1, AP_RND, AP_SAT> dot_t;       // skalarni prd   [-1, 1),  25 frac bita
typedef ap_fixed <26, 2, AP_RND, AP_SAT> norm_t;      // normirani vekt [-2, 2),  24 frac bita
typedef ap_fixed <20, 3, AP_RND, AP_SAT> accel_t;     // akcelerometar  [-4, 4),  17 frac bita
typedef ap_fixed <26, 2, AP_RND, AP_SAT> hg_t;        // halfGravity    [-2, 2),  24 frac bita
typedef ap_ufixed<22, 2, AP_RND, AP_SAT> magsq_t;     // magnituda²     [ 0, 4),  20 frac bita
typedef ap_ufixed<24, 6, AP_RND, AP_SAT> invmag_t;    // inv-mag        [ 0,64),  18 frac bita
typedef ap_fixed <22, 2, AP_RND, AP_SAT> halfgyro_t;  // polu-žiro      [-2, 2),  20 frac bita
typedef ap_ufixed<20, 4, AP_RND, AP_SAT> gain_t;      // pojačanje      [ 0,16),  16 frac bita
typedef ap_fixed <26, 5, AP_RND, AP_SAT> adjhg_t;     // adj polu-žiro  [-16,16), 21 frac bita
typedef ap_fixed <27, 5, AP_RND, AP_SAT> dq_t;        // delta-kvaternion,        22 frac bita
typedef ap_ufixed<20, 0, AP_RND, AP_SAT> dt_t;        // deltaTime      [ 0, 1),  20 frac bita

// ============================================================================
// Fast inverse square root
// INLINE off → HLS instancira kao dijeljeni modul, oba poziva dijele iste DSP-e
// BIND_OP fabric → float množenja idu na LUT, ne na DSP48E1
// ============================================================================
static float fast_inv_sqrt_hls(float x)
{
#pragma HLS INLINE off
#pragma HLS BIND_OP variable=return op=fmul impl=fabric
    union { float f; int32_t i; } u;
    u.f = x;
    u.i = 0x5F1F1412 - (u.i >> 1);
    float nr = 1.69000231f - 0.714158168f * x * u.f * u.f;
#pragma HLS BIND_OP variable=nr op=fmul impl=fabric
    return u.f * nr;
}

// ============================================================================
// ahrs_ip_top – HLS top-level funkcija, 1 uzorak po pozivu
//
// Ulaz:  kvaternion prethodnog uzorka (normalizovan od SW Sec.9) +
//        svi podaci jednog uzorka (gx..gz, ax..az, halfGravity, dt, rampedGain)
// Izlaz: nenormalizovani kvaternion (SW Sec.9 normalizuje)
// ============================================================================
void ahrs_ip_top(
    // Kvaternion ulaz (iz prethodnog uzorka, normalizovan)
    float  q0_in,  float  q1_in,  float  q2_in,  float  q3_in,
    // Podaci uzorka
    float  gx,     float  gy,     float  gz,
    float  ax,     float  ay,     float  az,
    float  hgX,    float  hgY,    float  hgZ,
    float  deltaTime,
    float  rampedGain_f,
    // Kvaternion izlaz (nenormalizovan)
    float &q0_out, float &q1_out, float &q2_out, float &q3_out)
{
#pragma HLS INTERFACE ap_ctrl_hs port=return
#pragma HLS INTERFACE ap_none    port=q0_in
#pragma HLS INTERFACE ap_none    port=q1_in
#pragma HLS INTERFACE ap_none    port=q2_in
#pragma HLS INTERFACE ap_none    port=q3_in
#pragma HLS INTERFACE ap_none    port=gx
#pragma HLS INTERFACE ap_none    port=gy
#pragma HLS INTERFACE ap_none    port=gz
#pragma HLS INTERFACE ap_none    port=ax
#pragma HLS INTERFACE ap_none    port=ay
#pragma HLS INTERFACE ap_none    port=az
#pragma HLS INTERFACE ap_none    port=hgX
#pragma HLS INTERFACE ap_none    port=hgY
#pragma HLS INTERFACE ap_none    port=hgZ
#pragma HLS INTERFACE ap_none    port=deltaTime
#pragma HLS INTERFACE ap_none    port=rampedGain_f
#pragma HLS INTERFACE ap_vld     port=q0_out
#pragma HLS INTERFACE ap_vld     port=q1_out
#pragma HLS INTERFACE ap_vld     port=q2_out
#pragma HLS INTERFACE ap_vld     port=q3_out

    const float DEG_TO_HALF_RAD = 3.14159265358979f / 360.0f;

    // Kvaternion u fixed-point
    const quat_t q0 = q0_in;
    const quat_t q1 = q1_in;
    const quat_t q2 = q2_in;
    const quat_t q3 = q3_in;

    // ── SEKCIJA 5: Povratna sprega akcelerometra ──────────────────────────────
    norm_t halfFeedbackX = 0.0f;
    norm_t halfFeedbackY = 0.0f;
    norm_t halfFeedbackZ = 0.0f;

    const bool accelNotZero = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));
    if (accelNotZero) {
        const accel_t ax_fp = ax;
        const accel_t ay_fp = ay;
        const accel_t az_fp = az;

        const hg_t hgX_fp = hgX;
        const hg_t hgY_fp = hgY;
        const hg_t hgZ_fp = hgZ;

        const magsq_t  accelMagSq  = ax_fp * ax_fp + ay_fp * ay_fp + az_fp * az_fp;
        const invmag_t accelInvMag = fast_inv_sqrt_hls((float)accelMagSq);

        const norm_t nx = ax_fp * accelInvMag;
        const norm_t ny = ay_fp * accelInvMag;
        const norm_t nz = az_fp * accelInvMag;

        norm_t crossX = ny * hgZ_fp - nz * hgY_fp;
        norm_t crossY = nz * hgX_fp - nx * hgZ_fp;
        norm_t crossZ = nx * hgY_fp - ny * hgX_fp;

        const dot_t dot = nx * hgX_fp + ny * hgY_fp + nz * hgZ_fp;
        if ((float)dot < 0.0f) {
            const magsq_t  crossMagSq  = crossX * crossX + crossY * crossY + crossZ * crossZ;
            const invmag_t crossInvMag = fast_inv_sqrt_hls((float)crossMagSq);
            crossX = crossX * crossInvMag;
            crossY = crossY * crossInvMag;
            crossZ = crossZ * crossInvMag;
        }

        halfFeedbackX = crossX;
        halfFeedbackY = crossY;
        halfFeedbackZ = crossZ;
    }

    // ── SEKCIJA 6: Žiroskop deg/s → polu-ugao rad/s ──────────────────────────
    const halfgyro_t halfGx = gx * DEG_TO_HALF_RAD;
    const halfgyro_t halfGy = gy * DEG_TO_HALF_RAD;
    const halfgyro_t halfGz = gz * DEG_TO_HALF_RAD;

    // ── SEKCIJA 7: Primjena pojačanja na povratnu spregu ──────────────────────
    const gain_t  rampedGain = rampedGain_f;
    const adjhg_t adjHalfGx = halfGx + halfFeedbackX * rampedGain;
    const adjhg_t adjHalfGy = halfGy + halfFeedbackY * rampedGain;
    const adjhg_t adjHalfGz = halfGz + halfFeedbackZ * rampedGain;

    // ── SEKCIJA 8: Integracija kvaterniona ────────────────────────────────────
    const dt_t dt = deltaTime;

    const dq_t dqw = -q1 * adjHalfGx - q2 * adjHalfGy - q3 * adjHalfGz;
    const dq_t dqx =  q0 * adjHalfGx + q2 * adjHalfGz - q3 * adjHalfGy;
    const dq_t dqy =  q0 * adjHalfGy - q1 * adjHalfGz + q3 * adjHalfGx;
    const dq_t dqz =  q0 * adjHalfGz + q1 * adjHalfGy - q2 * adjHalfGx;

    q0_out = (float)(q0 + dqw * dt);
    q1_out = (float)(q1 + dqx * dt);
    q2_out = (float)(q2 + dqy * dt);
    q3_out = (float)(q3 + dqz * dt);
}
