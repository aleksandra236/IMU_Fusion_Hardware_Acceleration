/**
 * @file tb_ahrs_ip_hls.cpp
 * @brief Testbench za Vitis HLS C Simulation
 *
 * Pokrece jedan uzorak kroz ahrs_ip_hls() i poredi sa float referencom.
 * Vraca 0 ako je greska u okviru tolerancije, 1 ako nije.
 *
 */

#include "ahrs_ip_hls.hpp"
#include <stdio.h>
#include <math.h>

// Tolerancija – greska do 0.01 po komponenti je prihvatljiva za 18-bit quat_t
#define MAX_ERROR 0.01f

// ============================================================================
// Float referenca – ista logika kao ahrs_hw.cpp, ali u cistu float aritmetiku
// Koristimo je da proverimo da li ahrs_ip_hls daje isti rezultat
// ============================================================================
static void reference_sections5to8(
    float qw, float qx, float qy, float qz,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float hgX, float hgY, float hgZ,
    float dt,  float gain,
    float *out_qw, float *out_qx, float *out_qy, float *out_qz)
{
    // Sekcija 5
    float fbX = 0.0f, fbY = 0.0f, fbZ = 0.0f;
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        float magSq = ax*ax + ay*ay + az*az;
        // fast inv sqrt
        union { float f; int i; } u = {magSq};
        u.i = 0x5F1F1412 - (u.i >> 1);
        float inv = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);
        float nax = ax*inv, nay = ay*inv, naz = az*inv;

        float cx = nay*hgZ - naz*hgY;
        float cy = naz*hgX - nax*hgZ;
        float cz = nax*hgY - nay*hgX;

        float dot = nax*hgX + nay*hgY + naz*hgZ;
        if (dot < 0.0f) {
            float cSq = cx*cx + cy*cy + cz*cz;
            union { float f; int i; } uc = {cSq};
            uc.i = 0x5F1F1412 - (uc.i >> 1);
            float ci = uc.f * (1.69000231f - 0.714158168f * cSq * uc.f * uc.f);
            cx *= ci; cy *= ci; cz *= ci;
        }
        fbX = cx; fbY = cy; fbZ = cz;
    }

    // Sekcija 6
    const float D2HR = 3.14159265358979f / 360.0f;
    float hgx = gx * D2HR;
    float hgy = gy * D2HR;
    float hgz = gz * D2HR;

    // Sekcija 7
    float agx = hgx + fbX * gain;
    float agy = hgy + fbY * gain;
    float agz = hgz + fbZ * gain;

    // Sekcija 8
    *out_qw = qw + (-qx*agx - qy*agy - qz*agz) * dt;
    *out_qx = qx + ( qw*agx + qy*agz - qz*agy) * dt;
    *out_qy = qy + ( qw*agy - qx*agz + qz*agx) * dt;
    *out_qz = qz + ( qw*agz + qx*agy - qy*agx) * dt;
}

// ============================================================================
// main – Vitis HLS testbench entry point
// ============================================================================
int main(void)
{
    // Testni ulazi – reprezentativne vrednosti iz sensor_data_short.csv
    // (prvi uzorak, inicijalno stanje kvaterniona)
    const float qw_f = 1.0f, qx_f = 0.0f, qy_f = 0.0f, qz_f = 0.0f;
    const float gx   =  2.05f,  gy  =  0.51f,  gz  =  0.12f;
    const float ax   =  0.01f,  ay  =  0.02f,  az  =  1.00f;
    const float hgX  =  0.00f,  hgY =  0.50f,  hgZ =  0.50f;
    const float dt   =  0.01f;
    const float gain = 10.0f;

    // Poziv HLS funkcije
    quat_t qw_in = qw_f, qx_in = qx_f, qy_in = qy_f, qz_in = qz_f;
    quat_t qw_out, qx_out, qy_out, qz_out;

    ahrs_ip_hls(qw_in, qx_in, qy_in, qz_in,
                gx, gy, gz,
                ax, ay, az,
                hgX, hgY, hgZ,
                dt, gain,
                qw_out, qx_out, qy_out, qz_out);

    // Float referenca
    float ref_qw, ref_qx, ref_qy, ref_qz;
    reference_sections5to8(qw_f, qx_f, qy_f, qz_f,
                           gx, gy, gz,
                           ax, ay, az,
                           hgX, hgY, hgZ,
                           dt, gain,
                           &ref_qw, &ref_qx, &ref_qy, &ref_qz);

    // Poredenje
    float ew = fabsf((float)qw_out - ref_qw);
    float ex = fabsf((float)qx_out - ref_qx);
    float ey = fabsf((float)qy_out - ref_qy);
    float ez = fabsf((float)qz_out - ref_qz);

    printf("HLS     : qw=%.6f  qx=%.6f  qy=%.6f  qz=%.6f\n",
           (float)qw_out, (float)qx_out, (float)qy_out, (float)qz_out);
    printf("Ref     : qw=%.6f  qx=%.6f  qy=%.6f  qz=%.6f\n",
           ref_qw, ref_qx, ref_qy, ref_qz);
    printf("Greska  : ew=%.6f  ex=%.6f  ey=%.6f  ez=%.6f\n",
           ew, ex, ey, ez);

    int pass = (ew < MAX_ERROR) && (ex < MAX_ERROR) &&
               (ey < MAX_ERROR) && (ez < MAX_ERROR);

    if (pass)
        printf("\nRezultat: OK\n");
    else
        printf("\nRezultat: GRESKA – jedna ili vise komponenti prelaze toleranciju %.4f\n",
               MAX_ERROR);

    return pass ? 0 : 1;
}
