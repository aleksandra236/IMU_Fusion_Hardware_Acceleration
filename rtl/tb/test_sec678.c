/**
 * test_sec678.c
 *
 * C referenca za VHDL modul ahrs_sec678 (Sekcije 6, 7, 8).
 * Koristi ISKLJUCIVO celobrojnu aritmetiku identičnu VHDL-u.
 *
 * Compile:  gcc -o test_sec678 test_sec678.c -lm
 * Run:      ./test_sec678
 *
 * Porediti izlaz sa Vivado simulacijom tb_ahrs_sec678.vhd
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* ============================================================================
 * Formatiranje fixed-point u float (za citljiv ispis)
 * ========================================================================= */
static double fp_to_double(int64_t raw, int frac_bits) {
    return (double)raw / (1LL << frac_bits);
}

/* ============================================================================
 * Konverzija float -> Q9.7 (16-bit signed)  - za gx,gy,gz ulaze
 * ========================================================================= */
static int16_t float_to_q9_7(double val) {
    return (int16_t)round(val * 128.0);
}

/* ============================================================================
 * Konverzija float -> Q4.16 (20-bit unsigned) - za gain
 * ========================================================================= */
static uint32_t float_to_q4_16(double val) {
    return (uint32_t)round(val * 65536.0);
}

/* ============================================================================
 * Konverzija float -> Q2.16 (18-bit signed) - za quaternion
 * ========================================================================= */
static int32_t float_to_q2_16(double val) {
    return (int32_t)round(val * 65536.0);
}

/* ============================================================================
 * Konverzija float -> Q0.20 (20-bit unsigned) - za dt
 * ========================================================================= */
static uint32_t float_to_q0_20(double val) {
    return (uint32_t)round(val * 1048576.0);
}

/* ============================================================================
 * ahrs_sec678_fp: Sekcije 6, 7, 8 u celobrojnoj aritmetici
 *
 * Formats (identični VHDL-u):
 *   gx,gy,gz  : signed 16-bit Q9.7   (range [-256, 256) deg/s)
 *   hfb*      : signed 32-bit Q2.24  (range [-2, 2), sek.5 izlaz)
 *   gain      : unsigned 32-bit Q4.16 (range [0, 16))
 *   q*        : signed 32-bit Q2.16  (range [-2, 2))
 *   dt        : unsigned 32-bit Q0.20 (range [0, 1))
 *   q*_new    : signed 32-bit Q2.16  (izlaz, nenormalizovan)
 * ========================================================================= */
void ahrs_sec678_fp(
    int16_t  gx,   int16_t  gy,   int16_t  gz,
    int32_t  hfbx, int32_t  hfby, int32_t  hfbz,
    uint32_t gain,
    int32_t  qw,   int32_t  qx,   int32_t  qy,   int32_t  qz,
    uint32_t dt,
    int32_t *qw_new, int32_t *qx_new, int32_t *qy_new, int32_t *qz_new
) {
    /* pi/360 u Q1.20 = round(pi/360 * 2^20) = 9150 */
    const int32_t C_PI360 = 9150;

    printf("\n======== SEKCIJA 6: Gyro konverzija ========\n");
    /* -----------------------------------------------------------------------
     * Sec.6: halfGyro = gx * (pi/360)
     * gx Q9.7 (16-bit) x C_PI360 Q1.20 (21-bit) -> Q10.27 (37-bit)
     * Izvuci Q2.20 (22-bit): uzmi bits[28:7] = product >> 7, keep 22 bits
     * --------------------------------------------------------------------- */
    int64_t prod6x = (int64_t)gx * C_PI360;
    int64_t prod6y = (int64_t)gy * C_PI360;
    int64_t prod6z = (int64_t)gz * C_PI360;

    /* bits[28:7] = aritmeticko pomeranje desno za 7, zadrzati 22 bita */
    int32_t hgx = (int32_t)(prod6x >> 7) & 0x3FFFFF;
    int32_t hgy = (int32_t)(prod6y >> 7) & 0x3FFFFF;
    int32_t hgz = (int32_t)(prod6z >> 7) & 0x3FFFFF;

    /* Znak (22-bit -> signed 32-bit sign extension) */
    if (hgx & 0x200000) hgx |= 0xFFC00000;
    if (hgy & 0x200000) hgy |= 0xFFC00000;
    if (hgz & 0x200000) hgz |= 0xFFC00000;

    printf("  gx_raw=%d (%.6f deg/s)\n", gx, fp_to_double(gx, 7));
    printf("  gy_raw=%d (%.6f deg/s)\n", gy, fp_to_double(gy, 7));
    printf("  gz_raw=%d (%.6f deg/s)\n", gz, fp_to_double(gz, 7));
    printf("  prod6x=%lld, prod6y=%lld, prod6z=%lld  [Q10.27]\n",
           (long long)prod6x, (long long)prod6y, (long long)prod6z);
    printf("  hgx=%d (%.6f), hgy=%d (%.6f), hgz=%d (%.6f)  [Q2.20]\n",
           hgx, fp_to_double(hgx, 20),
           hgy, fp_to_double(hgy, 20),
           hgz, fp_to_double(hgz, 20));

    printf("\n======== SEKCIJA 7: Primena feedback-a ========\n");
    /* -----------------------------------------------------------------------
     * Sec.7: adjHalfGyro = halfGyro + halfAccelFeedback * rampedGain
     *
     * Deo A: hfb Q2.24 x gain Q4.16 -> Q6.40 (47-bit)
     *        Izvuci Q5.21: bits[44:19] = product >> 19
     *
     * Deo B: halfGyro Q2.20 -> Q5.21: pomeri levo za 1 (raw *= 2)
     *
     * Deo C: adjhg = hg_aligned + hfb_gain_contrib
     * --------------------------------------------------------------------- */
    int64_t gain_s = (int64_t)gain;  /* uvek >= 0 */

    int64_t prod7x = (int64_t)hfbx * gain_s;
    int64_t prod7y = (int64_t)hfby * gain_s;
    int64_t prod7z = (int64_t)hfbz * gain_s;

    int32_t contrib_x = (int32_t)((prod7x >> 19) & 0x3FFFFFF);
    int32_t contrib_y = (int32_t)((prod7y >> 19) & 0x3FFFFFF);
    int32_t contrib_z = (int32_t)((prod7z >> 19) & 0x3FFFFFF);
    if (contrib_x & 0x2000000) contrib_x |= 0xFC000000;
    if (contrib_y & 0x2000000) contrib_y |= 0xFC000000;
    if (contrib_z & 0x2000000) contrib_z |= 0xFC000000;

    /* Poravnanje hg Q2.20 -> Q5.21: raw * 2 (dodaj 1 frac bit, sign-extend) */
    int32_t hgx_a = hgx * 2;
    int32_t hgy_a = hgy * 2;
    int32_t hgz_a = hgz * 2;

    int32_t adjgx = hgx_a + contrib_x;
    int32_t adjgy = hgy_a + contrib_y;
    int32_t adjgz = hgz_a + contrib_z;

    printf("  hfb_raw=[%d, %d, %d]  gain_raw=%u\n", hfbx, hfby, hfbz, gain);
    printf("  contrib = [%d, %d, %d]  [Q5.21, hfb*gain deo]\n",
           contrib_x, contrib_y, contrib_z);
    printf("  hg_aligned = [%d, %d, %d]  [Q5.21, pomeren levo za 1]\n",
           hgx_a, hgy_a, hgz_a);
    printf("  adjgx=%d (%.6f), adjgy=%d (%.6f), adjgz=%d (%.6f)  [Q5.21]\n",
           adjgx, fp_to_double(adjgx, 21),
           adjgy, fp_to_double(adjgy, 21),
           adjgz, fp_to_double(adjgz, 21));

    printf("\n======== SEKCIJA 8a: Kvaternionska derivacija ========\n");
    /* -----------------------------------------------------------------------
     * Sec.8a: dq iz q x adjHalfGyro
     *   dqw = -(qx*adjgx + qy*adjgy + qz*adjgz)
     *   dqx =  qw*adjgx + qy*adjgz - qz*adjgy
     *   dqy =  qw*adjgy - qx*adjgz + qz*adjgx
     *   dqz =  qw*adjgz + qx*adjgy - qy*adjgx
     *
     * q Q2.16 (18-bit) x adjhg Q5.21 (26-bit) -> Q7.37 (44-bit)
     * Sumu radimo u 46-bit akumulatoru (max 3 proizvoda ~96, treba 7 int bita)
     * Izvuci Q5.22 (27-bit): bits[41:15] = sum >> 15
     * --------------------------------------------------------------------- */
    int64_t p00 = (int64_t)qw * adjgx;
    int64_t p01 = (int64_t)qw * adjgy;
    int64_t p02 = (int64_t)qw * adjgz;
    int64_t p10 = (int64_t)qx * adjgx;
    int64_t p11 = (int64_t)qx * adjgy;
    int64_t p12 = (int64_t)qx * adjgz;
    int64_t p20 = (int64_t)qy * adjgx;
    int64_t p21 = (int64_t)qy * adjgy;
    int64_t p22 = (int64_t)qy * adjgz;
    int64_t p30 = (int64_t)qz * adjgx;
    int64_t p31 = (int64_t)qz * adjgy;
    int64_t p32 = (int64_t)qz * adjgz;

    int64_t sum_w = -(p10 + p21 + p32);
    int64_t sum_x =   p00 + p22 - p31;
    int64_t sum_y =   p01 - p12 + p30;
    int64_t sum_z =   p02 + p11 - p20;

    /* bits[41:15] = >> 15, zadrzati 27 bita */
    int32_t dqw = (int32_t)((sum_w >> 15) & 0x7FFFFFF);
    int32_t dqx = (int32_t)((sum_x >> 15) & 0x7FFFFFF);
    int32_t dqy = (int32_t)((sum_y >> 15) & 0x7FFFFFF);
    int32_t dqz = (int32_t)((sum_z >> 15) & 0x7FFFFFF);
    if (dqw & 0x4000000) dqw |= 0xF8000000;
    if (dqx & 0x4000000) dqx |= 0xF8000000;
    if (dqy & 0x4000000) dqy |= 0xF8000000;
    if (dqz & 0x4000000) dqz |= 0xF8000000;

    printf("  qw=%d, qx=%d, qy=%d, qz=%d  [Q2.16]\n", qw, qx, qy, qz);
    printf("  sum_w=%lld, sum_x=%lld, sum_y=%lld, sum_z=%lld  [Q7.37]\n",
           (long long)sum_w, (long long)sum_x,
           (long long)sum_y, (long long)sum_z);
    printf("  dqw=%d (%.6f), dqx=%d (%.6f)  [Q5.22]\n",
           dqw, fp_to_double(dqw, 22), dqx, fp_to_double(dqx, 22));
    printf("  dqy=%d (%.6f), dqz=%d (%.6f)  [Q5.22]\n",
           dqy, fp_to_double(dqy, 22), dqz, fp_to_double(dqz, 22));

    printf("\n======== SEKCIJA 8b: Eulerova integracija ========\n");
    /* -----------------------------------------------------------------------
     * Sec.8b: q_new = q + dq * dt
     *
     * dq Q5.22 (27-bit) x dt Q0.20 (20-bit unsigned) -> Q5.42 (48-bit)
     * Izvuci Q2.16 (18-bit): bits[43:26] = product >> 26
     *
     * q_new = q + contribution  (oba Q2.16, 18-bit)
     * --------------------------------------------------------------------- */
    int64_t dt_s = (int64_t)dt;

    int64_t pw = (int64_t)dqw * dt_s;
    int64_t px = (int64_t)dqx * dt_s;
    int64_t py = (int64_t)dqy * dt_s;
    int64_t pz = (int64_t)dqz * dt_s;

    int32_t cw = (int32_t)((pw >> 26) & 0x3FFFF);
    int32_t cx = (int32_t)((px >> 26) & 0x3FFFF);
    int32_t cy = (int32_t)((py >> 26) & 0x3FFFF);
    int32_t cz = (int32_t)((pz >> 26) & 0x3FFFF);
    if (cw & 0x20000) cw |= 0xFFFC0000;
    if (cx & 0x20000) cx |= 0xFFFC0000;
    if (cy & 0x20000) cy |= 0xFFFC0000;
    if (cz & 0x20000) cz |= 0xFFFC0000;

    printf("  dt_raw=%u (%.6f s)\n", dt, fp_to_double(dt, 20));
    printf("  dq*dt contrib: cw=%d, cx=%d, cy=%d, cz=%d  [Q2.16]\n",
           cw, cx, cy, cz);

    *qw_new = qw + cw;
    *qx_new = qx + cx;
    *qy_new = qy + cy;
    *qz_new = qz + cz;

    printf("\n======== IZLAZ Q_NEW (Q2.16) ========\n");
    printf("  qw_new = %d  (%.6f)\n", *qw_new, fp_to_double(*qw_new, 16));
    printf("  qx_new = %d  (%.6f)\n", *qx_new, fp_to_double(*qx_new, 16));
    printf("  qy_new = %d  (%.6f)\n", *qy_new, fp_to_double(*qy_new, 16));
    printf("  qz_new = %d  (%.6f)\n", *qz_new, fp_to_double(*qz_new, 16));
}

/* ============================================================================
 * main: pokreni test za 2 uzorka
 * ========================================================================= */
int main(void) {
    printf("============================================================\n");
    printf("  AHRS Sec.678 Fixed-Point C Referenca\n");
    printf("  Porediti sa tb_ahrs_sec678.vhd simulacijom\n");
    printf("============================================================\n");

    /* Sensor podaci iz sensor_data_full.csv */
    const double SAMPLE_GX[] = { -2.031874, -1.567595, -0.742677 };
    const double SAMPLE_GY[] = {  1.029231,  0.046824,  1.770211 };
    const double SAMPLE_GZ[] = { -0.920331, -1.063642, -0.672013 };

    const double GAIN    = 10.0;
    const double DT      = 0.01;

    /* Pocetni kvaternion: identitet */
    int32_t qw = float_to_q2_16(1.0);
    int32_t qx = 0, qy = 0, qz = 0;

    printf("\n--- Ulazni fixed-point parametri ---\n");
    printf("  C_PI360 = 9150  (pi/360 u Q1.20)\n");
    printf("  gain_raw = %u  (%.1f u Q4.16)\n",
           float_to_q4_16(GAIN), GAIN);
    printf("  dt_raw   = %u  (%.3f u Q0.20)\n",
           float_to_q0_20(DT), DT);
    printf("  qw_raw=%d, qx_raw=%d, qy_raw=%d, qz_raw=%d  [Q2.16]\n\n",
           qw, qx, qy, qz);

    for (int i = 0; i < 3; i++) {
        int16_t  gx_raw  = float_to_q9_7(SAMPLE_GX[i]);
        int16_t  gy_raw  = float_to_q9_7(SAMPLE_GY[i]);
        int16_t  gz_raw  = float_to_q9_7(SAMPLE_GZ[i]);
        uint32_t gain_raw = float_to_q4_16(GAIN);
        uint32_t dt_raw  = float_to_q0_20(DT);

        printf("\n############################################################\n");
        printf("  UZORAK %d\n", i + 1);
        printf("############################################################\n");
        printf("  gx=%.6f -> %d Q9.7 (greska: %.6f deg/s)\n",
               SAMPLE_GX[i], gx_raw,
               SAMPLE_GX[i] - fp_to_double(gx_raw, 7));
        printf("  gy=%.6f -> %d Q9.7\n", SAMPLE_GY[i], gy_raw);
        printf("  gz=%.6f -> %d Q9.7\n", SAMPLE_GZ[i], gz_raw);

        int32_t qw_new, qx_new, qy_new, qz_new;
        ahrs_sec678_fp(
            gx_raw, gy_raw, gz_raw,
            0, 0, 0,          /* hfb = 0 (Sec.5 nije implementirana) */
            gain_raw,
            qw, qx, qy, qz,
            dt_raw,
            &qw_new, &qx_new, &qy_new, &qz_new
        );

        /* Float referenca (za poredjenje) */
        double hgx_f = SAMPLE_GX[i] * (M_PI / 360.0);
        double hgy_f = SAMPLE_GY[i] * (M_PI / 360.0);
        double hgz_f = SAMPLE_GZ[i] * (M_PI / 360.0);
        double qw_f  = fp_to_double(qw, 16);
        double qx_f  = fp_to_double(qx, 16);
        double qy_f  = fp_to_double(qy, 16);
        double qz_f  = fp_to_double(qz, 16);
        double dqw_f = -(qx_f*hgx_f + qy_f*hgy_f + qz_f*hgz_f);
        double dqx_f =   qw_f*hgx_f + qy_f*hgz_f - qz_f*hgy_f;
        double dqy_f =   qw_f*hgy_f - qx_f*hgz_f + qz_f*hgx_f;
        double dqz_f =   qw_f*hgz_f + qx_f*hgy_f - qy_f*hgx_f;
        double qw_ref = qw_f + dqw_f * DT;
        double qx_ref = qx_f + dqx_f * DT;
        double qy_ref = qy_f + dqy_f * DT;
        double qz_ref = qz_f + dqz_f * DT;

        printf("\n  --- Float referenca ---\n");
        printf("  qw_ref=%.6f, qx_ref=%.6f, qy_ref=%.6f, qz_ref=%.6f\n",
               qw_ref, qx_ref, qy_ref, qz_ref);
        printf("  Greska fixed vs float:\n");
        printf("    dqw=%.2e  dqx=%.2e  dqy=%.2e  dqz=%.2e\n",
               fp_to_double(qw_new, 16) - qw_ref,
               fp_to_double(qx_new, 16) - qx_ref,
               fp_to_double(qy_new, 16) - qy_ref,
               fp_to_double(qz_new, 16) - qz_ref);

        printf("\n  >>> KOPIRAJ OVE VREDNOSTI U TESTBENCH <<<\n");
        printf("  EXP_QW = to_signed(%d, 18);\n", qw_new);
        printf("  EXP_QX = to_signed(%d, 18);\n", qx_new);
        printf("  EXP_QY = to_signed(%d, 18);\n", qy_new);
        printf("  EXP_QZ = to_signed(%d, 18);\n", qz_new);

        /* Izlaz postaje ulaz za sledeci uzorak */
        qw = qw_new;
        qx = qx_new;
        qy = qy_new;
        qz = qz_new;
    }

    printf("\n============================================================\n");
    printf("  Kraj. Pokreni Vivado simulaciju i uporedi.\n");
    printf("============================================================\n");
    return 0;
}
