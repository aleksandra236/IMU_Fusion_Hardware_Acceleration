/**
 * @file ahrs_ip_hls.cpp
 * @brief AHRS Hardware Accelerator – Sekcije 5-8, Vitis HLS implementacija
 *
 * Kljucne pragme za smanjenje DSP zauzetosti:
 *
 *   ALLOCATION function instances=fast_inv_sqrt limit=1
 *     – oba poziva fast_inv_sqrt dele jednu HW instancu (vec probano, -3 DSP)
 *
 *   ALLOCATION operation instances=mul limit=30
 *     – ogranicava ukupan broj mnozilica na 30 instanci
 *     – HLS time-multipleksira sva mnozenja kroz 30 DSP48E1 blokova
 *     – tradeoff: veca latencija po uzorku, ali sample-by-sample @ 100Hz
 *       ima 10ms izmedju uzoraka pa latencija nije problem
 *     – ocekivani DSP: ~30 (mnozenja) + 5 (fast_inv_sqrt) = ~35 ukupno
 */

#include "ahrs_ip_hls.hpp"
#include <stdint.h>

#define DEG_TO_HALF_RAD  (3.14159265358979f / 360.0f)

// INLINE off – oba poziva dele jednu HW instancu (ne dve odvojene)
static float fast_inv_sqrt(float x)
{
#pragma HLS INLINE off
    union { float f; int32_t i; } u;
    u.f = x;
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

void ahrs_ip_hls(
    quat_t  qw_in,  quat_t  qx_in,  quat_t  qy_in,  quat_t  qz_in,
    float   gx,     float   gy,     float   gz,
    float   ax,     float   ay,     float   az,
    float   halfGravityX, float halfGravityY, float halfGravityZ,
    float   deltaTime,    float rampedGain,
    quat_t &qw_out, quat_t &qx_out, quat_t &qy_out, quat_t &qz_out
)
{
#pragma HLS INTERFACE ap_none port=qw_in
#pragma HLS INTERFACE ap_none port=qx_in
#pragma HLS INTERFACE ap_none port=qy_in
#pragma HLS INTERFACE ap_none port=qz_in
#pragma HLS INTERFACE ap_none port=gx
#pragma HLS INTERFACE ap_none port=gy
#pragma HLS INTERFACE ap_none port=gz
#pragma HLS INTERFACE ap_none port=ax
#pragma HLS INTERFACE ap_none port=ay
#pragma HLS INTERFACE ap_none port=az
#pragma HLS INTERFACE ap_none port=halfGravityX
#pragma HLS INTERFACE ap_none port=halfGravityY
#pragma HLS INTERFACE ap_none port=halfGravityZ
#pragma HLS INTERFACE ap_none port=deltaTime
#pragma HLS INTERFACE ap_none port=rampedGain
#pragma HLS INTERFACE ap_vld  port=qw_out
#pragma HLS INTERFACE ap_vld  port=qx_out
#pragma HLS INTERFACE ap_vld  port=qy_out
#pragma HLS INTERFACE ap_vld  port=qz_out

    // fast_inv_sqrt: oba poziva dele jednu HW instancu
#pragma HLS ALLOCATION function instances=fast_inv_sqrt limit=1

    // Kljucna pragma: ogranicava ukupan broj mnozilica na 30 DSP48E1 instanci.
    // HLS rasporedjuje sva ap_fixed mnozenja kroz tih 30 instanci sekvencijalno.
    // Bez ove pragme HLS instancira ~87 paralelnih mnozilica.
#pragma HLS ALLOCATION operation instances=mul limit=26

    const quat_t qw = qw_in;
    const quat_t qx = qx_in;
    const quat_t qy = qy_in;
    const quat_t qz = qz_in;

    // =========================================================================
    // SEKCIJA 5: Povratna sprega akcelerometra
    // =========================================================================

    float halfFeedbackX = 0.0f;
    float halfFeedbackY = 0.0f;
    float halfFeedbackZ = 0.0f;

    const bool accelNotZero = !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f));

    if (accelNotZero) {
        const accel_t ax_fp = ax;
        const accel_t ay_fp = ay;
        const accel_t az_fp = az;

        const hg_t hgX = halfGravityX;
        const hg_t hgY = halfGravityY;
        const hg_t hgZ = halfGravityZ;

        const magsq_t accelMagSq = ax_fp * ax_fp + ay_fp * ay_fp + az_fp * az_fp;
        const invmag_t accelInvMag = fast_inv_sqrt((float)accelMagSq);

        const norm_t normAx = ax_fp * accelInvMag;
        const norm_t normAy = ay_fp * accelInvMag;
        const norm_t normAz = az_fp * accelInvMag;

        norm_t crossX = normAy * hgZ - normAz * hgY;
        norm_t crossY = normAz * hgX - normAx * hgZ;
        norm_t crossZ = normAx * hgY - normAy * hgX;

        const dot_t dot = normAx * hgX + normAy * hgY + normAz * hgZ;
        if (dot < (dot_t)0.0f) {
            const magsq_t crossMagSq = crossX * crossX + crossY * crossY + crossZ * crossZ;
            const invmag_t crossInvMag = fast_inv_sqrt((float)crossMagSq);
            crossX = crossX * crossInvMag;
            crossY = crossY * crossInvMag;
            crossZ = crossZ * crossInvMag;
        }

        halfFeedbackX = (float)crossX;
        halfFeedbackY = (float)crossY;
        halfFeedbackZ = (float)crossZ;
    }

    // =========================================================================
    // SEKCIJA 6: Ziroskop deg/s → polu-ugaona brzina (rad/s x 0.5)
    // =========================================================================

    const halfgyro_t halfGx = gx * DEG_TO_HALF_RAD;
    const halfgyro_t halfGy = gy * DEG_TO_HALF_RAD;
    const halfgyro_t halfGz = gz * DEG_TO_HALF_RAD;

    // =========================================================================
    // SEKCIJA 7: Primena povratne sprege na ziro brzine
    // =========================================================================

    const gain_t  gain_fp   = rampedGain;
    const adjhg_t adjHalfGx = halfGx + adjhg_t(halfFeedbackX) * gain_fp;
    const adjhg_t adjHalfGy = halfGy + adjhg_t(halfFeedbackY) * gain_fp;
    const adjhg_t adjHalfGz = halfGz + adjhg_t(halfFeedbackZ) * gain_fp;

    // =========================================================================
    // SEKCIJA 8: Integracija kvaterniona (prvi red)
    // =========================================================================

    const dt_t dt = deltaTime;

    const dq_t dqw = -qx * adjHalfGx - qy * adjHalfGy - qz * adjHalfGz;
    const dq_t dqx =  qw * adjHalfGx + qy * adjHalfGz - qz * adjHalfGy;
    const dq_t dqy =  qw * adjHalfGy - qx * adjHalfGz + qz * adjHalfGx;
    const dq_t dqz =  qw * adjHalfGz + qx * adjHalfGy - qy * adjHalfGx;

    qw_out = qw + quat_t(dqw * dt);
    qx_out = qx + quat_t(dqx * dt);
    qy_out = qy + quat_t(dqy * dt);
    qz_out = qz + quat_t(dqz * dt);
}
