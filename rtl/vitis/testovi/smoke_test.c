/* =============================================================================
 * smoke_test.c  -  Dijagnostički test za AHRS IP
 *
 * Cilj: utvrditi DA LI IP UOPŠTE RADI, korak po korak.
 *
 * Test 1: Da li BRAM read/write radi?
 * Test 2: Da li AHRS_STATUS može da se čita pre START-a?
 * Test 3: Da li IP prihvata START signal?
 * Test 4: Da li IP završava (done bit)?
 * Test 5: Sa stvarnim podacima — koliko traje sec5+sec678?
 * Test 6: Da li su output reči (qw,qx,qy,qz,hfbx,hfby,hfbz) razumne?
 *
 * Zameni hello_world.c ovim. Kompajliraj. Pokreni. Pošalji ceo izlaz.
 * ============================================================================= */

#include <stdio.h>
#include <stdint.h>
#include "xil_printf.h"
#include "xil_io.h"

#define AHRS_BASE       0x40000000U
#define BRAM_BASE       0x42000000U
#define AHRS_CTRL       (AHRS_BASE + 0x00U)
#define AHRS_STATUS     (AHRS_BASE + 0x04U)
#define STATUS_BUSY     0x1U
#define STATUS_DONE     0x2U

#define BRAM_IN_WORD    92U
#define BRAM_OUT_WORD   202U
#define BRAM_IN_BYTE    (BRAM_BASE + BRAM_IN_WORD * 4U)
#define BRAM_OUT_BYTE   (BRAM_BASE + BRAM_OUT_WORD * 4U)

int main(void) {
    xil_printf("\r\n========== AHRS IP SMOKE TEST ==========\r\n");

    /* ==================================================================
     * TEST 1: BRAM read/write sanity
     * ================================================================== */
    xil_printf("\r\n[TEST 1] BRAM read/write sanity\r\n");
    Xil_Out32(BRAM_BASE + 0U, 0xDEADBEEFU);
    Xil_Out32(BRAM_BASE + 4U, 0x12345678U);
    Xil_Out32(BRAM_BASE + 8U, 0xCAFEBABEU);
    uint32_t r1 = Xil_In32(BRAM_BASE + 0U);
    uint32_t r2 = Xil_In32(BRAM_BASE + 4U);
    uint32_t r3 = Xil_In32(BRAM_BASE + 8U);
    xil_printf("  Wrote 0xDEADBEEF, read 0x%08lX  %s\r\n",
               (unsigned long)r1, (r1 == 0xDEADBEEFU) ? "OK" : "FAIL");
    xil_printf("  Wrote 0x12345678, read 0x%08lX  %s\r\n",
               (unsigned long)r2, (r2 == 0x12345678U) ? "OK" : "FAIL");
    xil_printf("  Wrote 0xCAFEBABE, read 0x%08lX  %s\r\n",
               (unsigned long)r3, (r3 == 0xCAFEBABEU) ? "OK" : "FAIL");

    if (r1 != 0xDEADBEEFU) {
        xil_printf("  *** BRAM ne radi! Address mapping problem ili clock problem.\r\n");
        xil_printf("  *** Daljnji testovi nemaju smisla.\r\n");
        return -1;
    }

    /* ==================================================================
     * TEST 2: AHRS_STATUS pre svakog START-a
     * ================================================================== */
    xil_printf("\r\n[TEST 2] AHRS_STATUS pre START-a\r\n");
    uint32_t st0 = Xil_In32(AHRS_STATUS);
    xil_printf("  AHRS_STATUS pre START = 0x%08lX  (busy=%u, done=%u)\r\n",
               (unsigned long)st0,
               (unsigned)((st0 & STATUS_BUSY) ? 1 : 0),
               (unsigned)((st0 & STATUS_DONE) ? 1 : 0));
    if (st0 == 0xFFFFFFFFU) {
        xil_printf("  *** STATUS = 0xFFFFFFFF znači AXI nije dostupan!\r\n");
        return -2;
    }

    /* ==================================================================
     * TEST 3: Postavi MINIMALNE ulaze, daj START, prati STATUS
     * ================================================================== */
    xil_printf("\r\n[TEST 3] Postavljam ulaze (sve nule osim qw=1.0, az=-1.0)\r\n");

    /* gx, gy, gz - sve nule (Q9.7) */
    Xil_Out32(BRAM_IN_BYTE +  0U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  1U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  2U * 4U, 0x00000000U);

    /* ax=0, ay=0, az=-1.0 (IEEE 754 float) */
    Xil_Out32(BRAM_IN_BYTE +  3U * 4U, 0x00000000U);  /* +0.0 */
    Xil_Out32(BRAM_IN_BYTE +  4U * 4U, 0x00000000U);  /* +0.0 */
    Xil_Out32(BRAM_IN_BYTE +  5U * 4U, 0xBF800000U);  /* -1.0 */

    /* hgx=0, hgy=0, hgz=0.5 (IEEE 754 float) */
    Xil_Out32(BRAM_IN_BYTE +  6U * 4U, 0x00000000U);  /* +0.0 */
    Xil_Out32(BRAM_IN_BYTE +  7U * 4U, 0x00000000U);  /* +0.0 */
    Xil_Out32(BRAM_IN_BYTE +  8U * 4U, 0x3F000000U);  /* +0.5 */

    /* dt = 0.01 (Q0.20) = 10485 */
    Xil_Out32(BRAM_IN_BYTE +  9U * 4U, 10485U);

    /* gain = 0.5 (Q4.16) = 32768 */
    Xil_Out32(BRAM_IN_BYTE + 10U * 4U, 32768U);

    /* qw=1.0, qx=qy=qz=0 (Q2.16) */
    Xil_Out32(BRAM_IN_BYTE + 11U * 4U, 65536U);
    Xil_Out32(BRAM_IN_BYTE + 12U * 4U, 0U);
    Xil_Out32(BRAM_IN_BYTE + 13U * 4U, 0U);
    Xil_Out32(BRAM_IN_BYTE + 14U * 4U, 0U);

    /* Očisti output region tako da znamo da li HW upisuje */
    for (int i = 0; i < 7; i++) {
        Xil_Out32(BRAM_OUT_BYTE + i * 4U, 0xAAAAAAAAU);
    }

    /* Verifikacija da su ulazi stigli u BRAM */
    xil_printf("  Verifikacija ulaza:\r\n");
    xil_printf("    word 11 (qw) = 0x%08lX  (treba 0x00010000)\r\n",
               (unsigned long)Xil_In32(BRAM_IN_BYTE + 11U * 4U));
    xil_printf("    word  5 (az) = 0x%08lX  (treba 0xBF800000)\r\n",
               (unsigned long)Xil_In32(BRAM_IN_BYTE +  5U * 4U));
    xil_printf("    word  9 (dt) = 0x%08lX  (treba 0x000028F5)\r\n",
               (unsigned long)Xil_In32(BRAM_IN_BYTE +  9U * 4U));

    /* Verifikacija da je output pre starta = 0xAAAAAAAA */
    uint32_t out_before = Xil_In32(BRAM_OUT_BYTE + 0U * 4U);
    xil_printf("    output[0] pre starta = 0x%08lX  (treba 0xAAAAAAAA)\r\n",
               (unsigned long)out_before);

    /* DAJ START */
    xil_printf("\r\n  Pisem START u AHRS_CTRL...\r\n");
    Xil_Out32(AHRS_CTRL, 0x1U);

    /* Sampling STATUS u različitim trenucima */
    uint32_t st_a = Xil_In32(AHRS_STATUS);  /* odmah */
    for (volatile int j = 0; j < 100; j++);
    uint32_t st_b = Xil_In32(AHRS_STATUS);  /* posle 100 cycles */
    for (volatile int j = 0; j < 1000; j++);
    uint32_t st_c = Xil_In32(AHRS_STATUS);  /* posle 1100 cycles */
    for (volatile int j = 0; j < 10000; j++);
    uint32_t st_d = Xil_In32(AHRS_STATUS);  /* posle 11100 cycles */
    for (volatile int j = 0; j < 100000; j++);
    uint32_t st_e = Xil_In32(AHRS_STATUS);  /* posle 110000 cycles */
    for (volatile int j = 0; j < 1000000; j++);
    uint32_t st_f = Xil_In32(AHRS_STATUS);  /* posle 1.1M cycles */

    xil_printf("\r\n  STATUS sample (cycles ~ posle START):\r\n");
    xil_printf("    odmah:     0x%08lX  busy=%u done=%u\r\n",
               (unsigned long)st_a, (st_a & STATUS_BUSY) ? 1U : 0U, (st_a & STATUS_DONE) ? 1U : 0U);
    xil_printf("    ~100:      0x%08lX  busy=%u done=%u\r\n",
               (unsigned long)st_b, (st_b & STATUS_BUSY) ? 1U : 0U, (st_b & STATUS_DONE) ? 1U : 0U);
    xil_printf("    ~1100:     0x%08lX  busy=%u done=%u\r\n",
               (unsigned long)st_c, (st_c & STATUS_BUSY) ? 1U : 0U, (st_c & STATUS_DONE) ? 1U : 0U);
    xil_printf("    ~11100:    0x%08lX  busy=%u done=%u\r\n",
               (unsigned long)st_d, (st_d & STATUS_BUSY) ? 1U : 0U, (st_d & STATUS_DONE) ? 1U : 0U);
    xil_printf("    ~111K:     0x%08lX  busy=%u done=%u\r\n",
               (unsigned long)st_e, (st_e & STATUS_BUSY) ? 1U : 0U, (st_e & STATUS_DONE) ? 1U : 0U);
    xil_printf("    ~1.1M:     0x%08lX  busy=%u done=%u\r\n",
               (unsigned long)st_f, (st_f & STATUS_BUSY) ? 1U : 0U, (st_f & STATUS_DONE) ? 1U : 0U);

    /* ==================================================================
     * TEST 4: Pročitaj output bez obzira na done
     * ================================================================== */
    xil_printf("\r\n[TEST 4] Output reči (bez obzira na DONE):\r\n");
    for (int i = 0; i < 7; i++) {
        uint32_t v = Xil_In32(BRAM_OUT_BYTE + i * 4U);
        const char *name = "?";
        switch (i) {
            case 0: name = "qw  "; break;
            case 1: name = "qx  "; break;
            case 2: name = "qy  "; break;
            case 3: name = "qz  "; break;
            case 4: name = "hfbx"; break;
            case 5: name = "hfby"; break;
            case 6: name = "hfbz"; break;
        }
        xil_printf("    word %d (%s) = 0x%08lX\r\n", i, name, (unsigned long)v);
    }

    /* ==================================================================
     * INTERPRETACIJA
     * ================================================================== */
    xil_printf("\r\n========== INTERPRETACIJA ==========\r\n");

    if ((st_a & STATUS_DONE) || (st_b & STATUS_DONE)) {
        xil_printf("ZAKLJUCAK: DONE odmah ili posle 100 ciklusa.\r\n");
        xil_printf("  -> AXI kontrola radi.\r\n");
        xil_printf("  -> sec5to8 brzo zavrsava. Proveri output reci.\r\n");
    } else if (st_f & STATUS_DONE) {
        xil_printf("ZAKLJUCAK: DONE konacno postavljen posle ~1.1M ciklusa.\r\n");
        xil_printf("  -> Wrapper FSM funkcionise.\r\n");
        xil_printf("  -> Proveri output reci ispod.\r\n");
    } else if (st_f & STATUS_BUSY) {
        xil_printf("ZAKLJUCAK: BUSY=1, DONE=0 cak i posle 1.1M ciklusa.\r\n");
        xil_printf("  -> ahrs_sec5to8 NIKAD NE VRACA ready=1.\r\n");
        xil_printf("  -> sec5 ne zavrsava (FP IP problem) ILI\r\n");
        xil_printf("  -> sec678_start nikad ne pulsira (start_pending bag) ILI\r\n");
        xil_printf("  -> sec678 ne zavrsava (valid prop bag).\r\n");
    } else if ((st_f & (STATUS_BUSY | STATUS_DONE)) == 0) {
        xil_printf("ZAKLJUCAK: BUSY=0, DONE=0 - IP NIKAD NIJE NI STARTOVAN!\r\n");
        xil_printf("  -> AHRS_CTRL write ne stize do IP-a.\r\n");
        xil_printf("  -> Proveri AXI Lite address mapping u Address Editor.\r\n");
        xil_printf("  -> Ili je verzija ahrs_ip stara u block design-u.\r\n");
    }

    xil_printf("\r\n========== KRAJ TEST ==========\r\n");
    return 0;
}
