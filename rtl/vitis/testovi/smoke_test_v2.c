/* =============================================================================
 * smoke_test_v2.c  -  Fokusiran test: da li IP UOPSTE pise u BRAM?
 *
 * V1 je pokazao: BUSY=1 odmah posle START-a, ali posle 100 ciklusa BUSY=0.
 * Ovo znaci da je FSM otisao u S_DONE -> S_IDLE BRZO, ali CPU propusti
 * DONE puls od 1 ciklusa. ILI je FSM otisao u 'when others' bez pisanja.
 *
 * Ovaj test:
 *  1. Postavi output reci na 0xAAAAAAAA (poznata vrednost)
 *  2. Postavi ulaze
 *  3. Daj START
 *  4. ODMAH cita output reci NEKOLIKO puta uzastopno
 *  5. Pokazi kako se output rec menja kroz vreme
 *
 * Ako output ikad postane != 0xAAAAAAAA -> IP pise -> wrapper FSM radi
 * Ako output ostaje 0xAAAAAAAA zauvek -> wrapper FSM ne dolazi do S_WRITE_ISSUE
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

int main(void) {
    xil_printf("\r\n===== SMOKE V2: Output watcher =====\r\n");

    /* Postavi ulaze - sve nule osim qw=1.0, az=-1.0 */
    Xil_Out32(BRAM_IN_BYTE +  0U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  1U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  2U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  3U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  4U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  5U * 4U, 0xBF800000U);
    Xil_Out32(BRAM_IN_BYTE +  6U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  7U * 4U, 0x00000000U);
    Xil_Out32(BRAM_IN_BYTE +  8U * 4U, 0x3F000000U);
    Xil_Out32(BRAM_IN_BYTE +  9U * 4U, 10485U);
    Xil_Out32(BRAM_IN_BYTE + 10U * 4U, 32768U);
    Xil_Out32(BRAM_IN_BYTE + 11U * 4U, 65536U);
    Xil_Out32(BRAM_IN_BYTE + 12U * 4U, 0U);
    Xil_Out32(BRAM_IN_BYTE + 13U * 4U, 0U);
    Xil_Out32(BRAM_IN_BYTE + 14U * 4U, 0U);

    /* Markiraj output reci sa 0xAAAAAAAA */
    for (int i = 0; i < 7; i++) {
        Xil_Out32(BRAM_OUT_BYTE + i * 4U, 0xAAAAAAAAU);
    }

    xil_printf("Pre START: output[0]=0x%08lX\r\n",
               (unsigned long)Xil_In32(BRAM_OUT_BYTE + 0U * 4U));

    /* START! */
    Xil_Out32(AHRS_CTRL, 0x1U);

    /* CITAJ ODMAH output 50 puta uzastopno, bez ikakvog kasnjenja */
    xil_printf("\r\nDump output[0..6] kroz vreme (50 ocitavanja, ~5 cycles svako):\r\n");
    xil_printf("%-6s %10s %10s %10s %10s %10s %10s %10s %10s\r\n",
               "iter", "STATUS", "w0", "w1", "w2", "w3", "w4", "w5", "w6");

    for (int iter = 0; iter < 50; iter++) {
        uint32_t st = Xil_In32(AHRS_STATUS);
        uint32_t w0 = Xil_In32(BRAM_OUT_BYTE + 0U * 4U);
        uint32_t w1 = Xil_In32(BRAM_OUT_BYTE + 1U * 4U);
        uint32_t w2 = Xil_In32(BRAM_OUT_BYTE + 2U * 4U);
        uint32_t w3 = Xil_In32(BRAM_OUT_BYTE + 3U * 4U);
        uint32_t w4 = Xil_In32(BRAM_OUT_BYTE + 4U * 4U);
        uint32_t w5 = Xil_In32(BRAM_OUT_BYTE + 5U * 4U);
        uint32_t w6 = Xil_In32(BRAM_OUT_BYTE + 6U * 4U);
        xil_printf("%-6d 0x%08lX 0x%08lX 0x%08lX 0x%08lX 0x%08lX 0x%08lX 0x%08lX 0x%08lX\r\n",
                   iter,
                   (unsigned long)st,
                   (unsigned long)w0, (unsigned long)w1,
                   (unsigned long)w2, (unsigned long)w3,
                   (unsigned long)w4, (unsigned long)w5, (unsigned long)w6);
    }

    /* Sacekaj jos dosta, i citaj jos jednom */
    for (volatile int j = 0; j < 5000000; j++) {}
    xil_printf("\r\nPosle 5M wait cycles:\r\n");
    uint32_t st_f = Xil_In32(AHRS_STATUS);
    xil_printf("  STATUS = 0x%08lX\r\n", (unsigned long)st_f);
    for (int i = 0; i < 7; i++) {
        xil_printf("  output[%d] = 0x%08lX\r\n",
                   i, (unsigned long)Xil_In32(BRAM_OUT_BYTE + i * 4U));
    }

    /* TEST 2: Ponovo START i smesta polling STATUS hiljadu puta */
    xil_printf("\r\n===== TEST 2: Polling STATUS 1000 puta =====\r\n");
    for (int i = 0; i < 7; i++) {
        Xil_Out32(BRAM_OUT_BYTE + i * 4U, 0xAAAAAAAAU);
    }
    Xil_Out32(AHRS_CTRL, 0x1U);

    int saw_busy = 0;
    int saw_done = 0;
    int saw_both_zero = 0;
    for (int i = 0; i < 1000; i++) {
        uint32_t st = Xil_In32(AHRS_STATUS);
        if (st & 1) saw_busy++;
        if (st & 2) saw_done++;
        if (st == 0) saw_both_zero++;
    }
    xil_printf("Od 1000 ocitavanja STATUS-a:\r\n");
    xil_printf("  busy=1: %d puta\r\n", saw_busy);
    xil_printf("  done=1: %d puta\r\n", saw_done);
    xil_printf("  oba 0:  %d puta\r\n", saw_both_zero);

    xil_printf("\r\nFinal output:\r\n");
    for (int i = 0; i < 7; i++) {
        xil_printf("  output[%d] = 0x%08lX\r\n",
                   i, (unsigned long)Xil_In32(BRAM_OUT_BYTE + i * 4U));
    }

    xil_printf("\r\n===== KRAJ V2 =====\r\n");
    return 0;
}
