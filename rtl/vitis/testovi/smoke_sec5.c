/* =============================================================================
 * smoke_test_sec5.c  -  Testira sec5 sa 3 raznih ulaza
 *
 * Za svaki ulaz znamo PRAVU vrednost hfb (Madgwick formula).
 * Uporedi sa HW.
 * ============================================================================= */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
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

static int run_one(const char *name,
                   float ax, float ay, float az,
                   float hgx, float hgy, float hgz,
                   float exp_hfbx, float exp_hfby, float exp_hfbz)
{
    xil_printf("\r\n=== %s ===\r\n", name);
    xil_printf("Ulazi: ax=");pf(ax);xil_printf(" ay=");pf(ay);xil_printf(" az=");pf(az);xil_printf("\r\n");
    xil_printf("       hgx=");pf(hgx);xil_printf(" hgy=");pf(hgy);xil_printf(" hgz=");pf(hgz);xil_printf("\r\n");

    /* Postavi sve ostale ulaze na 0 */
    for (int i = 0; i < 15; i++) Xil_Out32(BRAM_IN_BYTE + i*4U, 0);

    /* Postavi accel i hg (Q9.7 gx/gy/gz su sve 0) */
    Xil_Out32(BRAM_IN_BYTE +  3U * 4U, flt2u32(ax));
    Xil_Out32(BRAM_IN_BYTE +  4U * 4U, flt2u32(ay));
    Xil_Out32(BRAM_IN_BYTE +  5U * 4U, flt2u32(az));
    Xil_Out32(BRAM_IN_BYTE +  6U * 4U, flt2u32(hgx));
    Xil_Out32(BRAM_IN_BYTE +  7U * 4U, flt2u32(hgy));
    Xil_Out32(BRAM_IN_BYTE +  8U * 4U, flt2u32(hgz));

    /* dt, gain, quat = 0 */
    Xil_Out32(BRAM_IN_BYTE +  9U * 4U, 10485U);   /* dt = 0.01 */
    Xil_Out32(BRAM_IN_BYTE + 10U * 4U, 32768U);   /* gain = 0.5 */
    Xil_Out32(BRAM_IN_BYTE + 11U * 4U, 65536U);   /* qw = 1.0 */

    /* Označi output */
    for (int i = 0; i < 7; i++) Xil_Out32(BRAM_OUT_BYTE + i*4U, 0xAAAAAAAAU);

    /* Pokreni */
    Xil_Out32(AHRS_CTRL, 0x1U);

    int timeout = 1000000;
    while (timeout-- > 0) {
        if (Xil_In32(AHRS_STATUS) & 0x2U) break;
    }
    if (timeout <= 0) {
        xil_printf("TIMEOUT!\r\n");
        return -1;
    }

    /* Citaj rezultat */
    int32_t r4 = (int32_t)Xil_In32(BRAM_OUT_BYTE + 4U * 4U);
    int32_t r5 = (int32_t)Xil_In32(BRAM_OUT_BYTE + 5U * 4U);
    int32_t r6 = (int32_t)Xil_In32(BRAM_OUT_BYTE + 6U * 4U);

    float hfbx_hw = (float)r4 / 16777216.0f;
    float hfby_hw = (float)r5 / 16777216.0f;
    float hfbz_hw = (float)r6 / 16777216.0f;

    xil_printf("HW raw:    hfbx=0x%08lX hfby=0x%08lX hfbz=0x%08lX\r\n",
               (unsigned long)r4, (unsigned long)r5, (unsigned long)r6);
    xil_printf("HW float:  hfbx=");pf(hfbx_hw);xil_printf(" hfby=");pf(hfby_hw);xil_printf(" hfbz=");pf(hfbz_hw);xil_printf("\r\n");
    xil_printf("Expected:  hfbx=");pf(exp_hfbx);xil_printf(" hfby=");pf(exp_hfby);xil_printf(" hfbz=");pf(exp_hfbz);xil_printf("\r\n");

    return 0;
}

int main(void) {
    xil_printf("\r\n===== SEC5 ISOLATION TEST =====\r\n");

    /* TEST A: accel i hg paralelni -> hfb = 0 */
    run_one("TEST A: parallel (a=hg)",
            0.0f, 0.0f, -1.0f,
            0.0f, 0.0f, 0.5f,
            0.0f, 0.0f, 0.0f);

    /* TEST B: accel = (1,0,0), hg = (0,0,0.5) -> cross dovodi do specifične vrednosti
     * na = (1,0,0), hg = (0,0,0.5)
     * dot = 0 -> NEMA normalizacije
     * c = na × hg = (0*0.5-0*0, 0*0-1*0.5, 1*0-0*0) = (0, -0.5, 0)
     * Posle dot >= 0 grane, vraca direktno c (bez norm)
     * hfb = (0, -0.5, 0)
     */
    run_one("TEST B: cross product, no norm (dot=0)",
            1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.5f,
            0.0f, -0.5f, 0.0f);

    /* TEST C: accel = (0.1, 0.2, -0.95), hg = (0, 0, 0.5)
     * |a| = sqrt(0.01+0.04+0.9025) = sqrt(0.9525) = 0.976
     * na = (0.1024, 0.2048, -0.9733)
     * c = na × hg = (0.2048*0.5 - (-0.9733)*0, (-0.9733)*0 - 0.1024*0.5, 0.1024*0 - 0.2048*0)
     *   = (0.1024, -0.0512, 0)
     * dot = 0.1024*0 + 0.2048*0 + (-0.9733)*0.5 = -0.4867
     * dot < 0 -> NORMALIZUJ
     * |c| = sqrt(0.01049 + 0.00262 + 0) = sqrt(0.01311) = 0.1145
     * c_norm = (0.894, -0.447, 0)
     * hfb = (0.894, -0.447, 0)
     */
    run_one("TEST C: cross + norm (dot<0)",
            0.1f, 0.2f, -0.95f,
            0.0f, 0.0f, 0.5f,
            0.894427f, -0.447214f, 0.0f);

    /* TEST D: accel = (0, 0.3, -0.95), hg = (0.1, 0.2, 0.4)
     * |a| = sqrt(0+0.09+0.9025) = 0.996
     * na = (0, 0.301, -0.954)
     * c = na × hg = (0.301*0.4 - (-0.954)*0.2,  (-0.954)*0.1 - 0*0.4,  0*0.2 - 0.301*0.1)
     *   = (0.1204 + 0.1908, -0.0954, -0.0301)
     *   = (0.3112, -0.0954, -0.0301)
     * dot = 0*0.1 + 0.301*0.2 + (-0.954)*0.4 = 0.0602 - 0.3816 = -0.3214
     * dot < 0 -> NORMALIZUJ
     * |c| = sqrt(0.0969 + 0.0091 + 0.000906) = sqrt(0.1069) = 0.3270
     * c_norm = (0.9517, -0.2917, -0.0921)
     */
    run_one("TEST D: full 3D normalized",
            0.0f, 0.3f, -0.95f,
            0.1f, 0.2f, 0.4f,
            0.9517f, -0.2917f, -0.0921f);

    xil_printf("\r\n===== KRAJ =====\r\n");
    return 0;
}
