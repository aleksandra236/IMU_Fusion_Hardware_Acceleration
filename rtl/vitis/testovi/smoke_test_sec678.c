/* =============================================================================
 * smoke_test_sec678.c  -  Izolacioni test za SEC678
 *
 * Postavi POZNATE ulaze (gyro, hfb, gain, dt, q_in) i vidi šta sec678 vrati.
 *
 * Za svaki test ručno smo izračunali šta TREBA da bude q_out.
 * Ako se HW slaže — sec678 RADI. Ako ne — bug je u sec678.
 *
 * VAŽNO: sec5 i sec678 dele isti IP. Sec5 računa hfb iz accel+hg.
 * Da bismo izolovali sec678, koristimo accel=(0,0,0) → sec5 short-circuit
 * → hfb=(0,0,0). Tako sec678 dobija hfb=0 i jedino što utiče je gyro.
 *
 * Onda za "sec678 + feedback" test, koristimo realne accel vrednosti
 * koje će dati hfb sa poznatim ručnim izračunom.
 * ============================================================================= */

#include <stdio.h>
#include <stdint.h>
#include "xil_printf.h"
#include "xil_io.h"

#define AHRS_BASE       0x40000000U
#define BRAM_BASE       0x42000000U
#define AHRS_CTRL       (AHRS_BASE + 0x00U)
#define AHRS_STATUS     (AHRS_BASE + 0x04U)
#define BRAM_IN_BYTE    (BRAM_BASE + 92U * 4U)
#define BRAM_OUT_BYTE   (BRAM_BASE + 202U * 4U)

static inline uint32_t flt2u32(float f) {
    union { float f; uint32_t u; } u; u.f = f; return u.u;
}

static void pf(float v) {
    if (v < 0.0f) { xil_printf("-"); v = -v; }
    int whole = (int)v;
    float fpart = (v - (float)whole) * 1000000.0f;
    int frac = (int)(fpart + 0.5f);
    xil_printf("%d.%06d", whole, frac);
}

/* Konvertuj float -> Q9.7 signed 16-bit, smesti u donja 16 bita */
static uint32_t flt_to_q9_7(float gx) {
    int32_t v = (int32_t)(gx * 128.0f);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (uint32_t)(v & 0xFFFF);
}

/* Konvertuj float -> Q2.16 signed 18-bit, smesti u donja 18 bita */
static uint32_t flt_to_q2_16(float q) {
    int32_t v = (int32_t)(q * 65536.0f);
    if (v > 131071) v = 131071;
    if (v < -131072) v = -131072;
    return (uint32_t)(v & 0x3FFFF);
}

static int run_sec678(const char *name,
                      float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float hgx, float hgy, float hgz,
                      float gain, float dt,
                      float qw_in, float qx_in, float qy_in, float qz_in,
                      float exp_qw, float exp_qx, float exp_qy, float exp_qz)
{
    xil_printf("\r\n=== %s ===\r\n", name);
    xil_printf("Gyro: gx=");pf(gx);xil_printf(" gy=");pf(gy);xil_printf(" gz=");pf(gz);xil_printf(" deg/s\r\n");
    xil_printf("Accel: ax=");pf(ax);xil_printf(" ay=");pf(ay);xil_printf(" az=");pf(az);xil_printf("\r\n");
    xil_printf("HG: hgx=");pf(hgx);xil_printf(" hgy=");pf(hgy);xil_printf(" hgz=");pf(hgz);xil_printf("\r\n");
    xil_printf("gain=");pf(gain);xil_printf(" dt=");pf(dt);xil_printf("\r\n");
    xil_printf("q_in: ");pf(qw_in);xil_printf(", ");pf(qx_in);xil_printf(", ");pf(qy_in);xil_printf(", ");pf(qz_in);xil_printf("\r\n");

    /* Postavi sve ulaze */
    Xil_Out32(BRAM_IN_BYTE +  0U * 4U, flt_to_q9_7(gx));
    Xil_Out32(BRAM_IN_BYTE +  1U * 4U, flt_to_q9_7(gy));
    Xil_Out32(BRAM_IN_BYTE +  2U * 4U, flt_to_q9_7(gz));
    Xil_Out32(BRAM_IN_BYTE +  3U * 4U, flt2u32(ax));
    Xil_Out32(BRAM_IN_BYTE +  4U * 4U, flt2u32(ay));
    Xil_Out32(BRAM_IN_BYTE +  5U * 4U, flt2u32(az));
    Xil_Out32(BRAM_IN_BYTE +  6U * 4U, flt2u32(hgx));
    Xil_Out32(BRAM_IN_BYTE +  7U * 4U, flt2u32(hgy));
    Xil_Out32(BRAM_IN_BYTE +  8U * 4U, flt2u32(hgz));
    Xil_Out32(BRAM_IN_BYTE +  9U * 4U, (uint32_t)(dt * 1048576.0f));        /* Q0.20 */
    Xil_Out32(BRAM_IN_BYTE + 10U * 4U, (uint32_t)(gain * 65536.0f));        /* Q4.16 */
    Xil_Out32(BRAM_IN_BYTE + 11U * 4U, flt_to_q2_16(qw_in));
    Xil_Out32(BRAM_IN_BYTE + 12U * 4U, flt_to_q2_16(qx_in));
    Xil_Out32(BRAM_IN_BYTE + 13U * 4U, flt_to_q2_16(qy_in));
    Xil_Out32(BRAM_IN_BYTE + 14U * 4U, flt_to_q2_16(qz_in));

    /* Označi output */
    for (int i = 0; i < 7; i++) Xil_Out32(BRAM_OUT_BYTE + i*4U, 0xAAAAAAAAU);

    /* Pokreni HW */
    Xil_Out32(AHRS_CTRL, 0x1U);

    int timeout = 1000000;
    while (timeout-- > 0) {
        if (Xil_In32(AHRS_STATUS) & 0x2U) break;
    }
    if (timeout <= 0) { xil_printf("TIMEOUT!\r\n"); return -1; }

    /* Citaj rezultat - Q2.16 signed 18-bit sign-extended u 32-bit */
    int32_t r_qw = (int32_t)Xil_In32(BRAM_OUT_BYTE + 0U * 4U);
    int32_t r_qx = (int32_t)Xil_In32(BRAM_OUT_BYTE + 1U * 4U);
    int32_t r_qy = (int32_t)Xil_In32(BRAM_OUT_BYTE + 2U * 4U);
    int32_t r_qz = (int32_t)Xil_In32(BRAM_OUT_BYTE + 3U * 4U);

    /* Sign extend Q2.16 (18-bit) u 32-bit */
    if (r_qw & 0x20000) r_qw |= 0xFFFC0000;
    if (r_qx & 0x20000) r_qx |= 0xFFFC0000;
    if (r_qy & 0x20000) r_qy |= 0xFFFC0000;
    if (r_qz & 0x20000) r_qz |= 0xFFFC0000;

    float qw_hw = (float)r_qw / 65536.0f;
    float qx_hw = (float)r_qx / 65536.0f;
    float qy_hw = (float)r_qy / 65536.0f;
    float qz_hw = (float)r_qz / 65536.0f;

    /* Procitaj i hfb (sec5 output - debug) */
    int32_t r_hx = (int32_t)Xil_In32(BRAM_OUT_BYTE + 4U * 4U);
    int32_t r_hy = (int32_t)Xil_In32(BRAM_OUT_BYTE + 5U * 4U);
    int32_t r_hz = (int32_t)Xil_In32(BRAM_OUT_BYTE + 6U * 4U);
    float hx_hw = (float)r_hx / 16777216.0f;
    float hy_hw = (float)r_hy / 16777216.0f;
    float hz_hw = (float)r_hz / 16777216.0f;

    xil_printf("HW hfb (sec5):  ");pf(hx_hw);xil_printf(", ");pf(hy_hw);xil_printf(", ");pf(hz_hw);xil_printf("\r\n");
    xil_printf("HW q_out:       qw=");pf(qw_hw);xil_printf(" qx=");pf(qx_hw);xil_printf(" qy=");pf(qy_hw);xil_printf(" qz=");pf(qz_hw);xil_printf("\r\n");
    xil_printf("Expected q_out: qw=");pf(exp_qw);xil_printf(" qx=");pf(exp_qx);xil_printf(" qy=");pf(exp_qy);xil_printf(" qz=");pf(exp_qz);xil_printf("\r\n");
    xil_printf("Razlika:        qw=");pf(qw_hw-exp_qw);xil_printf(" qx=");pf(qx_hw-exp_qx);xil_printf(" qy=");pf(qy_hw-exp_qy);xil_printf(" qz=");pf(qz_hw-exp_qz);xil_printf("\r\n");

    return 0;
}

int main(void) {
    xil_printf("\r\n===== SEC678 ISOLATION TEST =====\r\n");

    /* TEST 1: ČISTI GYRO oko X-ose, NEMA accel feedback (accel=0)
     *
     * Inputs:
     *   gx=10 deg/s, gy=gz=0
     *   accel=(0,0,0) → sec5 zero short-circuit → hfb=(0,0,0)
     *   gain=0, dt=0.01
     *   q_in = (1,0,0,0)
     *
     * Math:
     *   halfGyro_x = 10 * π/360 = 0.0873 rad/s
     *   adjg = halfGyro (jer gain*hfb = 0)
     *   dq = q × adjg = (qw=1) × (0.0873, 0, 0)
     *      dqw = 0
     *      dqx = qw*adjgx = 1*0.0873 = 0.0873
     *      dqy = 0
     *      dqz = 0
     *   q_new = q + dq*dt = (1, 0.0873*0.01, 0, 0) = (1, 0.000873, 0, 0)
     */
    run_sec678("TEST 1: pure gyro X (gain=0)",
        10.0f, 0.0f, 0.0f,           /* gyro */
        0.0f, 0.0f, 0.0f,            /* accel = 0 */
        0.0f, 0.0f, 0.0f,            /* hg = 0 (nebitno jer hfb=0) */
        0.0f, 0.01f,                  /* gain=0, dt=0.01 */
        1.0f, 0.0f, 0.0f, 0.0f,      /* q_in */
        1.0f, 0.000873f, 0.0f, 0.0f); /* expected q_out */

    /* TEST 2: ČISTI GYRO oko Y-ose */
    run_sec678("TEST 2: pure gyro Y (gain=0)",
        0.0f, 10.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.01f,
        1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.000873f, 0.0f);

    /* TEST 3: ČISTI GYRO oko Z-ose */
    run_sec678("TEST 3: pure gyro Z (gain=0)",
        0.0f, 0.0f, 10.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.01f,
        1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f, 0.000873f);

    /* TEST 4: GYRO X = 100 deg/s, gain=0
     * halfGyro_x = 100 * π/360 = 0.873
     * dqx = 1 * 0.873 = 0.873
     * qx_new = 0.873 * 0.01 = 0.00873
     */
    run_sec678("TEST 4: gyro X=100 deg/s (gain=0)",
        100.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.01f,
        1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.00873f, 0.0f, 0.0f);

    /* TEST 5: GYRO X = 100 deg/s, gain=1, accel zero → hfb=0
     * Isto kao TEST 4, čak i sa gain=1 jer hfb=0
     */
    run_sec678("TEST 5: gyro X=100 deg/s (gain=1, accel=0)",
        100.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        1.0f, 0.01f,
        1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.00873f, 0.0f, 0.0f);

    xil_printf("\r\n===== KRAJ =====\r\n");
    return 0;
}
