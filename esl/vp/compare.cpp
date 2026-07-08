/**
 * @file compare.cpp
 * @brief Float referenca vs VP (sc_fixed) – sample-by-sample verzija
 *
 * Workflow:
 *   1. Pokreni VP simulaciju:   make run   (produces vp_output.csv)
 *   2. Pokreni ovo:             make compare
 *
 * Float referenca replicira tacno VP logiku:
 *   - Za svaki uzorak: halfGravity iz trenutnog normalizovanog q
 *   - rampedGain se smanjuje po uzorku
 *   - HW sekcije 5-8 u float aritmetici (hw_q se NE resetuje izmedju uzoraka)
 *   - Normalizacija odmah posle svakog uzorka → azurira q
 */

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>

// ============================================================================
// Konfiguracija (mora da odgovara defines.hpp)
// ============================================================================
static const int   N_SAMPLES  = 1918;
static const float DT         = 0.01f;   // 100 Hz
static const float GAIN       = 0.5f;
static const char *SENSOR_CSV = "../../data/data1504.csv";
static const char *VP_CSV     = "vp_output.csv";
static const char *FLOAT_CSV  = "float_output.csv";
static const char *CMP_CSV    = "comparison_results.csv";

// ============================================================================
// Tipovi
// ============================================================================
struct Sample { float gx, gy, gz, ax, ay, az; };
struct Quat   { float w, x, y, z; };

// ============================================================================
// SW stanje – identično CPU klasi
// ============================================================================
static float q[4]         = {1.f, 0.f, 0.f, 0.f};
static float hw_q[4]      = {1.f, 0.f, 0.f, 0.f};  // NE resetuje se!
static float rampedGain   = 10.f;
static float rampedGainStep;
static bool  initialising = true;

// ============================================================================
// fast_inv_sqrt – identican specifikaciji
// ============================================================================
static inline float fast_inv_sqrt(float x)
{
    union { float f; int32_t i; } u = {x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

// ============================================================================
// SW Sekcije 1-4: priprema jednog uzorka
// Identično CPU::sw_prepare_sample()
// ============================================================================
static void sw_prepare(const Sample &s,
                       float &hgX, float &hgY, float &hgZ,
                       float &rg)
{
    // Sekcija 3: rampedGain
    if (initialising) {
        rampedGain -= rampedGainStep * DT;
        if (rampedGain < GAIN) {
            rampedGain   = GAIN;
            initialising = false;
        }
    }
    rg = rampedGain;

    // Sekcija 4: halfGravity iz TRENUTNOG q
    hgX = q[1]*q[2] - q[0]*q[1];  // qx*qz - qw*qy
    hgX = q[1]*q[3] - q[0]*q[2];
    hgY = q[2]*q[3] + q[0]*q[1];
    hgZ = q[0]*q[0] - 0.5f + q[3]*q[3];
}

// ============================================================================
// HW Sekcije 5-8 u float aritmetici
// hw_q se cuva i prenosi u sledeci poziv (sample-by-sample)
// ============================================================================
static Quat hw_step(const Sample &s,
                    float hgX, float hgY, float hgZ,
                    float rg)
{
    // Sekcija 5: povratna sprega akcelerometra
    float fbx = 0.f, fby = 0.f, fbz = 0.f;
    if (!((s.ax == 0.f) && (s.ay == 0.f) && (s.az == 0.f))) {
        const float magSq = s.ax*s.ax + s.ay*s.ay + s.az*s.az;
        const float inv   = fast_inv_sqrt(magSq);
        const float ax = s.ax*inv, ay = s.ay*inv, az = s.az*inv;

        float cx = ay*hgZ - az*hgY;
        float cy = az*hgX - ax*hgZ;
        float cz = ax*hgY - ay*hgX;

        const float dot = ax*hgX + ay*hgY + az*hgZ;
        if (dot < 0.f) {
            const float cmSq = cx*cx + cy*cy + cz*cz;
            const float ci   = fast_inv_sqrt(cmSq);
            cx *= ci;  cy *= ci;  cz *= ci;
        }
        fbx = cx;  fby = cy;  fbz = cz;
    }

    // Sekcija 6: ziroskop deg/s → polu-ugaona brzina
    const float D2HR = 3.14159265358979f / 360.f;
    const float halfGx = s.gx * D2HR;
    const float halfGy = s.gy * D2HR;
    const float halfGz = s.gz * D2HR;

    // Sekcija 7: primena povratne sprege
    const float adjX = halfGx + fbx * rg;
    const float adjY = halfGy + fby * rg;
    const float adjZ = halfGz + fbz * rg;

    // Sekcija 8: integracija kvaterniona (prvi red), nenormalizovano
    Quat out;
    out.w = hw_q[0] + (-hw_q[1]*adjX - hw_q[2]*adjY - hw_q[3]*adjZ) * DT;
    out.x = hw_q[1] + ( hw_q[0]*adjX + hw_q[2]*adjZ - hw_q[3]*adjY) * DT;
    out.y = hw_q[2] + ( hw_q[0]*adjY - hw_q[1]*adjZ + hw_q[3]*adjX) * DT;
    out.z = hw_q[3] + ( hw_q[0]*adjZ + hw_q[1]*adjY - hw_q[2]*adjX) * DT;

    // hw_q se azurira – prenosi se u sledeci uzorak (NE resetuje se)
    hw_q[0] = out.w;  hw_q[1] = out.x;
    hw_q[2] = out.y;  hw_q[3] = out.z;

    return out;
}

// ============================================================================
// SW Sekcije 9-11: normalizacija i zero-heading
// Identično CPU::sw_process_output()
// ============================================================================
static Quat sw_normalize(const Quat &unnorm)
{
    const float magSq = unnorm.w*unnorm.w + unnorm.x*unnorm.x
                      + unnorm.y*unnorm.y + unnorm.z*unnorm.z;
    union { float f; int32_t i; } u;
    u.f = magSq;
    u.i = 0x5F1F1412 - (u.i >> 1);
    const float inv = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);

    float qw = unnorm.w*inv, qx = unnorm.x*inv;
    float qy = unnorm.y*inv, qz = unnorm.z*inv;

    // Sekcija 10: zero-heading (samo tokom inicijalizacije)
    if (initialising) {
        const float yaw = atan2f(qw*qz + qx*qy, 0.5f - qy*qy - qz*qz);
        const float hy  = 0.5f * yaw;
        const float cy  = cosf(hy), sy = sinf(hy);
        float nw = cy*qw + sy*qz,  nx = cy*qx + sy*qy;
        float ny = cy*qy - sy*qx,  nz = cy*qz - sy*qw;
        qw = nw;  qx = nx;  qy = ny;  qz = nz;
    }

    // Azuriraj globalni q
    q[0] = qw;  q[1] = qx;  q[2] = qy;  q[3] = qz;
    return {qw, qx, qy, qz};
}

// ============================================================================
// Kvaternion → Ojlerovi uglovi (ZYX, stepeni)
// ============================================================================
static void quat_to_euler(const Quat &qt, float &roll, float &pitch, float &yaw)
{
    const float w = qt.w, x = qt.x, y = qt.y, z = qt.z;
    roll  = atan2f(2.f*(w*x + y*z), 1.f - 2.f*(x*x + y*y)) * (180.f / 3.14159265f);
    pitch = asinf (2.f*(w*y - z*x))                          * (180.f / 3.14159265f);
    yaw   = atan2f(2.f*(w*z + x*y), 1.f - 2.f*(y*y + z*z)) * (180.f / 3.14159265f);
}

// ============================================================================
// main
// ============================================================================
int main(void)
{
    rampedGainStep = (10.f - GAIN) / 3.f;

    // ── 1. Ucitaj uzorke ────────────────────────────────────────────────────
    FILE *f = fopen(SENSOR_CSV, "r");
    if (!f) { fprintf(stderr, "ERROR: ne mogu da otvorim %s\n", SENSOR_CSV); return 1; }

    char line[512];
    (void)fgets(line, sizeof(line), f);  // preskoči zaglavlje

    static Sample sensor_data[N_SAMPLES];
    int n_loaded = 0;
    while (fgets(line, sizeof(line), f) && n_loaded < N_SAMPLES) {
        float t, gx, gy, gz, ax, ay, az;
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f",
                   &t, &gx, &gy, &gz, &ax, &ay, &az) != 7) continue;
        sensor_data[n_loaded++] = {gx, gy, gz, ax, ay, az};
    }
    fclose(f);
    printf("Ucitano %d uzoraka iz %s\n", n_loaded, SENSOR_CSV);

    // ── 2. Float referenca – sample-by-sample ───────────────────────────────
    static Quat float_q[N_SAMPLES];

    for (int i = 0; i < n_loaded; i++) {
        float hgX, hgY, hgZ, rg;
        sw_prepare(sensor_data[i], hgX, hgY, hgZ, rg);
        Quat unnorm = hw_step(sensor_data[i], hgX, hgY, hgZ, rg);
        float_q[i]  = sw_normalize(unnorm);
    }

    // ── 3. Upisi float referencu ─────────────────────────────────────────────
    FILE *ff = fopen(FLOAT_CSV, "w");
    if (ff) {
        fprintf(ff, "sample,qw,qx,qy,qz\n");
        for (int i = 0; i < n_loaded; i++)
            fprintf(ff, "%d,%.8f,%.8f,%.8f,%.8f\n",
                    i, float_q[i].w, float_q[i].x, float_q[i].y, float_q[i].z);
        fclose(ff);
        printf("Float referenca upisana u %s\n", FLOAT_CSV);
    }

    // ── 4. Ucitaj VP izlaz ───────────────────────────────────────────────────
    static Quat vp_q[N_SAMPLES];
    int n_vp = 0;

    FILE *fv = fopen(VP_CSV, "r");
    if (!fv) {
        fprintf(stderr,
            "\nUPOZORENJE: %s nije nadjen.\n"
            "  Prvo pokreni './ahrs_vp', pa ponovi './compare'.\n\n", VP_CSV);
        return 0;
    }
    (void)fgets(line, sizeof(line), fv);
    while (fgets(line, sizeof(line), fv) && n_vp < N_SAMPLES) {
        int idx; float qw, qx, qy, qz;
        if (sscanf(line, "%d,%f,%f,%f,%f", &idx, &qw, &qx, &qy, &qz) == 5)
            vp_q[n_vp++] = {qw, qx, qy, qz};
    }
    fclose(fv);
    printf("Ucitano %d kvaterniona iz %s\n\n", n_vp, VP_CSV);

    // ── 5. Poredjenje ────────────────────────────────────────────────────────
    FILE *fc = fopen(CMP_CSV, "w");
    if (fc)
        fprintf(fc, "sample,"
                    "float_qw,float_qx,float_qy,float_qz,"
                    "vp_qw,vp_qx,vp_qy,vp_qz,"
                    "q_l2_error,"
                    "float_roll,float_pitch,float_yaw,"
                    "vp_roll,vp_pitch,vp_yaw,"
                    "err_roll,err_pitch,err_yaw\n");

    float max_q_err = 0.f, sum_q_err = 0.f;
    float max_roll  = 0.f, sum_roll  = 0.f;
    float max_pitch = 0.f, sum_pitch = 0.f;
    float max_yaw   = 0.f, sum_yaw   = 0.f;
    int   n_cmp = (n_loaded < n_vp) ? n_loaded : n_vp;

    printf("%-6s  %12s%12s%12s  |  %12s%12s%12s  |  %9s%9s%9s\n",
           "Uzorak", "Roll_F", "Pitch_F", "Yaw_F",
           "Roll_VP", "Pitch_VP", "Yaw_VP",
           "dRoll", "dPitch", "dYaw");
    printf("----------------------------------------------------------------------------------------\n");

    for (int i = 0; i < n_cmp; i++) {
        float dw = float_q[i].w - vp_q[i].w;
        float dx = float_q[i].x - vp_q[i].x;
        float dy = float_q[i].y - vp_q[i].y;
        float dz = float_q[i].z - vp_q[i].z;
        float q_err = sqrtf(dw*dw + dx*dx + dy*dy + dz*dz);
        sum_q_err += q_err;
        if (q_err > max_q_err) max_q_err = q_err;

        float fr, fp, fy, vr, vp, vy;
        quat_to_euler(float_q[i], fr, fp, fy);
        quat_to_euler(vp_q[i],    vr, vp, vy);

        float er = fabsf(fr - vr); if (er > 180.f) er = 360.f - er;
        float ep = fabsf(fp - vp); if (ep > 180.f) ep = 360.f - ep;
        float ey = fabsf(fy - vy); if (ey > 180.f) ey = 360.f - ey;

        sum_roll  += er; if (er > max_roll)  max_roll  = er;
        sum_pitch += ep; if (ep > max_pitch) max_pitch = ep;
        sum_yaw   += ey; if (ey > max_yaw)   max_yaw   = ey;

        if (i % 10 == 9)
            printf("%-6d  %+12.3f%+12.3f%+12.3f  |  %+12.3f%+12.3f%+12.3f  |  %9.4f%9.4f%9.4f\n",
                   i, fr, fp, fy, vr, vp, vy, er, ep, ey);

        if (fc)
            fprintf(fc, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
                        "%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    i,
                    float_q[i].w, float_q[i].x, float_q[i].y, float_q[i].z,
                    vp_q[i].w,    vp_q[i].x,    vp_q[i].y,    vp_q[i].z,
                    q_err, fr, fp, fy, vr, vp, vy, er, ep, ey);
    }
    if (fc) { fclose(fc); printf("\nDetaljno poredjenje upisano u %s\n", CMP_CSV); }

    // ── 6. Rezime ────────────────────────────────────────────────────────────
    printf("\n");
    printf("=============================================================\n");
    printf("  Rezime  (float ref vs VP sc_fixed,  %d uzoraka)\n", n_cmp);
    printf("=============================================================\n");
    printf("  Kvaternion L2 greska:  srednja = %.6f   maks = %.6f\n",
           sum_q_err / n_cmp, max_q_err);
    printf("  Roll  greska (stepen): srednja = %.4f    maks = %.4f\n",
           sum_roll  / n_cmp, max_roll);
    printf("  Pitch greska (stepen): srednja = %.4f    maks = %.4f\n",
           sum_pitch / n_cmp, max_pitch);
    printf("  Yaw   greska (stepen): srednja = %.4f    maks = %.4f\n",
           sum_yaw   / n_cmp, max_yaw);

    float fr, fp, fy, vr, vp2, vy;
    quat_to_euler(float_q[n_cmp-1], fr, fp, fy);
    quat_to_euler(vp_q[n_cmp-1],    vr, vp2, vy);
    printf("\n  Finalna orijentacija (uzorak %d):\n", n_cmp-1);
    printf("    %10s  %10s  %10s\n", "Roll", "Pitch", "Yaw");
    printf("    %+10.3f  %+10.3f  %+10.3f   (float ref)\n", fr, fp, fy);
    printf("    %+10.3f  %+10.3f  %+10.3f   (VP sc_fixed)\n", vr, vp2, vy);
    printf("    %+10.3f  %+10.3f  %+10.3f   (greska)\n", fr-vr, fp-vp2, fy-vy);
    printf("=============================================================\n\n");

    return 0;
}