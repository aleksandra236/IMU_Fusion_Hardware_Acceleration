/* =============================================================================
 * smoke_test_sec678_v2.c  -  Test sa TEST G1 (gain + accel)
 *
 * Funkcija run_sec678 prima accel/hg parametre eksplicitno.
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

static uint32_t flt_to_q9_7(float gx) {
    int32_t v = (int32_t)(gx * 128.0f);
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (uint32_t)(v & 0xFFFF);
}

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
    xil_printf("q_in: qw=");pf(qw_in);xil_printf(" qx=");pf(qx_in);xil_printf(" qy=");pf(qy_in);xil_printf(" qz=");pf(qz_in);xil_printf("\r\n");

    Xil_Out32(BRAM_IN_BYTE +  0U * 4U, flt_to_q9_7(gx));
    Xil_Out32(BRAM_IN_BYTE +  1U * 4U, flt_to_q9_7(gy));
    Xil_Out32(BRAM_IN_BYTE +  2U * 4U, flt_to_q9_7(gz));
    Xil_Out32(BRAM_IN_BYTE +  3U * 4U, flt2u32(ax));
    Xil_Out32(BRAM_IN_BYTE +  4U * 4U, flt2u32(ay));
    Xil_Out32(BRAM_IN_BYTE +  5U * 4U, flt2u32(az));
    Xil_Out32(BRAM_IN_BYTE +  6U * 4U, flt2u32(hgx));
    Xil_Out32(BRAM_IN_BYTE +  7U * 4U, flt2u32(hgy));
    Xil_Out32(BRAM_IN_BYTE +  8U * 4U, flt2u32(hgz));
    Xil_Out32(BRAM_IN_BYTE +  9U * 4U, (uint32_t)(dt * 1048576.0f));
    Xil_Out32(BRAM_IN_BYTE + 10U * 4U, (uint32_t)(gain * 65536.0f));
    Xil_Out32(BRAM_IN_BYTE + 11U * 4U, flt_to_q2_16(qw_in));
    Xil_Out32(BRAM_IN_BYTE + 12U * 4U, flt_to_q2_16(qx_in));
    Xil_Out32(BRAM_IN_BYTE + 13U * 4U, flt_to_q2_16(qy_in));
    Xil_Out32(BRAM_IN_BYTE + 14U * 4U, flt_to_q2_16(qz_in));

    for (int i = 0; i < 7; i++) Xil_Out32(BRAM_OUT_BYTE + i*4U, 0xAAAAAAAAU);

    Xil_Out32(AHRS_CTRL, 0x1U);

    int timeout = 1000000;
    while (timeout-- > 0) {
        if (Xil_In32(AHRS_STATUS) & 0x2U) break;
    }
    if (timeout <= 0) { xil_printf("TIMEOUT!\r\n"); return -1; }

    int32_t r_qw = (int32_t)Xil_In32(BRAM_OUT_BYTE + 0U * 4U);
    int32_t r_qx = (int32_t)Xil_In32(BRAM_OUT_BYTE + 1U * 4U);
    int32_t r_qy = (int32_t)Xil_In32(BRAM_OUT_BYTE + 2U * 4U);
    int32_t r_qz = (int32_t)Xil_In32(BRAM_OUT_BYTE + 3U * 4U);

    if (r_qw & 0x20000) r_qw |= 0xFFFC0000;
    if (r_qx & 0x20000) r_qx |= 0xFFFC0000;
    if (r_qy & 0x20000) r_qy |= 0xFFFC0000;
    if (r_qz & 0x20000) r_qz |= 0xFFFC0000;

    float qw_hw = (float)r_qw / 65536.0f;
    float qx_hw = (float)r_qx / 65536.0f;
    float qy_hw = (float)r_qy / 65536.0f;
    float qz_hw = (float)r_qz / 65536.0f;

    /* Hfb (sec5 izlaz) za debug */
    int32_t r_hx = (int32_t)Xil_In32(BRAM_OUT_BYTE + 4U * 4U);
    int32_t r_hy = (int32_t)Xil_In32(BRAM_OUT_BYTE + 5U * 4U);
    int32_t r_hz = (int32_t)Xil_In32(BRAM_OUT_BYTE + 6U * 4U);
    float hx = (float)r_hx / 16777216.0f;
    float hy = (float)r_hy / 16777216.0f;
    float hz = (float)r_hz / 16777216.0f;

    xil_printf("HW hfb:         ");pf(hx);xil_printf(", ");pf(hy);xil_printf(", ");pf(hz);xil_printf("\r\n");
    xil_printf("HW q_out:       qw=");pf(qw_hw);xil_printf(" qx=");pf(qx_hw);xil_printf(" qy=");pf(qy_hw);xil_printf(" qz=");pf(qz_hw);xil_printf("\r\n");
    xil_printf("Expected q_out: qw=");pf(exp_qw);xil_printf(" qx=");pf(exp_qx);xil_printf(" qy=");pf(exp_qy);xil_printf(" qz=");pf(exp_qz);xil_printf("\r\n");
    xil_printf("Razlika:        qw=");pf(qw_hw-exp_qw);xil_printf(" qx=");pf(qx_hw-exp_qx);xil_printf(" qy=");pf(qy_hw-exp_qy);xil_printf(" qz=");pf(qz_hw-exp_qz);xil_printf("\r\n");
    xil_printf("RAW qx hex: 0x%08lX (decimalno %ld)\r\n", (unsigned long)r_qx, (long)r_qx);

    return 0;
}

int main(void) {
    xil_printf("\r\n===== SEC678 TEST POSLE FIX-A =====\r\n");

    /* REF: gx=10, qw=1, dt=0.01, accel=0 (hfb=0)
     * Tacno: qx = 1 * (10*pi/360) * 0.01 = 0.000873
     */
    run_sec678("REF: gx=10, qw=1.0, dt=0.01, accel=0",
        10.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.01f,
        1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.000873f, 0.0f, 0.0f);

    /* TEST P: qw=0.5, accel=0 */
    run_sec678("TEST P: qw=0.5, accel=0",
        10.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.01f,
        0.5f, 0.0f, 0.0f, 0.0f,
        0.5f, 0.000437f, 0.0f, 0.0f);

    /* TEST Q: gx=0, qx_in=0.1, accel=0 (propagacija) */
    run_sec678("TEST Q: gx=0, qx_in=0.1, accel=0",
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.01f,
        1.0f, 0.1f, 0.0f, 0.0f,
        1.0f, 0.1f, 0.0f, 0.0f);

    /* TEST R: gx=10, qw=2.0, accel=0 */
    run_sec678("TEST R: qw=2.0, accel=0",
        10.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.01f,
        2.0f, 0.0f, 0.0f, 0.0f,
        2.0f, 0.001746f, 0.0f, 0.0f);

    /* TEST S: dt=0.005, accel=0 */
    run_sec678("TEST S: dt=0.005, accel=0",
        10.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.005f,
        1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.000437f, 0.0f, 0.0f);

    /* TEST G1: gain=10, accel=(0.1, 0.2, -0.95), hg=(0, 0, 0.5)
     * sec5 ce racunati hfb. Ocekivano hfb ≈ (0.894, -0.447, 0)
     * adjgx = halfGyroX + hfbx * gain = -0.0678 + 0.894*10 = 8.872
     * qx_out = qw * adjgx * dt = 1 * 8.872 * 0.01 = 0.08872
     */
    run_sec678("TEST G1: gain=10, accel real",
        10.0f, 0.0f, 0.0f,
        0.1f, 0.2f, -0.95f,
        0.0f, 0.0f, 0.5f,
        10.0f, 0.01f,
        1.0f, 0.0f, 0.0f, 0.0f,
        1.0f, 0.08872f, 0.0f, 0.0f);

    xil_printf("\r\n===== KRAJ =====\r\n");
    return 0;
}
