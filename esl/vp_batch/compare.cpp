/**
 * @file compare.cpp
 * @brief Float reference (pure float AHRS) vs VP (sc_fixed) output comparison
 *
 * Workflow:
 *   1.  Run the VP simulation first:    make run   (produces vp_output.csv)
 *   2.  Run this comparison tool:       make compare
 *
 * What it does:
 *   - Loads the first 200 samples from sensor_data_short.csv
 *   - Runs the same AHRS pipeline in pure float, replicating the VP's exact
 *   batch structure (batch_size = 5, halfGravity computed once per batch
 *     from the normalized quaternion at the END of the previous batch)
 *   - Writes the float reference output to float_output.csv
 *   - Reads vp_output.csv (produced by ./ahrs_vp)
 *   - Compares the two: quaternion L2 error, roll/pitch/yaw angle errors (°)
 *   - Writes the detailed comparison to comparison_results.csv
 *   - Prints a summary table and statistics to stdout
 *
 * Build: make compare   (see Makefile)
 */

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>

// ============================================================================
// Configuration  (must match defines.hpp)
// ============================================================================
static const int   BATCH_SIZE    = 5;
static const int   N_SAMPLES     = 200;          // first 200 samples
static const float DT            = 0.01f;        // 100 Hz → 10 ms
static const float GAIN          = 0.5f;
static const float GYRO_RANGE    = 2000.f;
static const char *SENSOR_CSV    = "../analysis/data/sensor_data_short.csv";
static const char *VP_CSV        = "vp_output.csv";
static const char *FLOAT_CSV     = "float_output.csv";
static const char *CMP_CSV       = "comparison_results.csv";

// ============================================================================
// Data types
// ============================================================================
struct Sample { float gx, gy, gz, ax, ay, az; };
struct Quat   { float w, x, y, z; };

// ============================================================================
// AHRS SW state (mirrors CPU member variables)
// ============================================================================
static float  q[4]          = {1.f, 0.f, 0.f, 0.f};  // [w, x, y, z]
static float  rampedGain    = 10.f;
static float  rampedGainStep;
static bool   initialising  = true;

// HW internal quaternion (IP state, resets to normalized q at batch start)
static float  hw_q[4] = {1.f, 0.f, 0.f, 0.f};

// ============================================================================
// Fast inverse square root (bit-magic – identical to spec)
// ============================================================================
static inline float fast_inv_sqrt(float x)
{
    union { float f; int32_t i; } u = {x};
    u.i = 0x5F1F1412 - (u.i >> 1);
    return u.f * (1.69000231f - 0.714158168f * x * u.f * u.f);
}

// ============================================================================
// HW Sections 5-8 in pure float
// Operates on hw_q[], feeds it forward (unnormalized) within a batch.
// ============================================================================
static Quat hw_step(const Sample &s,
                    float hgX, float hgY, float hgZ,
                    float rg)
{
    // SECTION 5: accelerometer feedback
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
        fbx = cx * rg;  fby = cy * rg;  fbz = cz * rg;
    }

    // SECTION 6: gyro deg/s → half-angle rad/s
    const float D2HR = 3.14159265358979f / 360.f;
    const float hx = s.gx * D2HR + fbx;
    const float hy = s.gy * D2HR + fby;
    const float hz = s.gz * D2HR + fbz;

    // SECTION 8: first-order quaternion integration (unnormalized)
    Quat out;
    out.w = hw_q[0] + (-hw_q[1]*hx - hw_q[2]*hy - hw_q[3]*hz) * DT;
    out.x = hw_q[1] + ( hw_q[0]*hx + hw_q[2]*hz - hw_q[3]*hy) * DT;
    out.y = hw_q[2] + ( hw_q[0]*hy - hw_q[1]*hz + hw_q[3]*hx) * DT;
    out.z = hw_q[3] + ( hw_q[0]*hz + hw_q[1]*hy - hw_q[2]*hx) * DT;

    hw_q[0] = out.w;  hw_q[1] = out.x;
    hw_q[2] = out.y;  hw_q[3] = out.z;
    return out;
}

// ============================================================================
// SW Sections 9-11: fast-inv-sqrt normalization + zero-heading
// Updates global q[] and syncs hw_q[] for the next batch start.
// ============================================================================
static Quat sw_normalize_and_update(const Quat &unnorm)
{
    const float magSq = unnorm.w*unnorm.w + unnorm.x*unnorm.x
                      + unnorm.y*unnorm.y + unnorm.z*unnorm.z;
    union { float f; int32_t i; } u;
    u.f = magSq;
    u.i = 0x5F1F1412 - (u.i >> 1);
    const float inv = u.f * (1.69000231f - 0.714158168f * magSq * u.f * u.f);

    float qw = unnorm.w*inv, qx = unnorm.x*inv;
    float qy = unnorm.y*inv, qz = unnorm.z*inv;

    // SECTION 10: zero-heading correction during initialisation
    if (initialising) {
        const float yaw = atan2f(qw*qz + qx*qy, 0.5f - qy*qy - qz*qz);
        const float hy  = 0.5f * yaw;
        const float cy  = cosf(hy), sy = sinf(hy);
        float nw = cy*qw + sy*qz,  nx = cy*qx + sy*qy;
        float ny = cy*qy - sy*qx,  nz = cy*qz - sy*qw;
        qw = nw;  qx = nx;  qy = ny;  qz = nz;
    }

    q[0] = qw;  q[1] = qx;  q[2] = qy;  q[3] = qz;
    return {qw, qx, qy, qz};
}

// ============================================================================
// Quaternion → Euler angles (ZYX convention, degrees)
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

    // ── 1. Load first N_SAMPLES from CSV ────────────────────────────────────
    FILE *f = fopen(SENSOR_CSV, "r");
    if (!f) {
        fprintf(stderr, "ERROR: cannot open sensor CSV: %s\n", SENSOR_CSV);
        return 1;
    }
    char line[512];
    (void)fgets(line, sizeof(line), f);  // skip header

    static Sample sensor_data[N_SAMPLES];
    int n_loaded = 0;
    while (fgets(line, sizeof(line), f) && n_loaded < N_SAMPLES) {
        float t, gx, gy, gz, ax, ay, az;
        if (sscanf(line, "%f,%f,%f,%f,%f,%f,%f",
                   &t, &gx, &gy, &gz, &ax, &ay, &az) != 7) continue;
        sensor_data[n_loaded++] = {gx, gy, gz, ax, ay, az};
    }
    fclose(f);

    if (n_loaded < N_SAMPLES) {
        fprintf(stderr, "WARNING: only %d samples in CSV (expected %d)\n",
                n_loaded, N_SAMPLES);
    }
    printf("Loaded %d sensor samples from %s\n", n_loaded, SENSOR_CSV);

    // ── 2. Run float reference (batch-aware, matches VP structure) ───────────
    static Quat float_q[N_SAMPLES];
    // batch_rg[i] stores the rampedGain used for sample i during Phase 1 prep
    static float batch_rg[N_SAMPLES];

    for (int b = 0; b < N_SAMPLES / BATCH_SIZE; b++) {
        const int base = b * BATCH_SIZE;

        // Compute halfGravity ONCE per batch from current normalized q[]
        // (mirrors CPU: all 10 sw_prepare_sample calls see the same q state)
        const float hgX = q[1]*q[3] - q[0]*q[2];
        const float hgY = q[2]*q[3] + q[0]*q[1];
        const float hgZ = q[0]*q[0] - 0.5f + q[3]*q[3];

        // Phase 1: advance rampedGain for each sample in the batch
        for (int i = 0; i < BATCH_SIZE; i++) {
            if (initialising) {
                rampedGain -= rampedGainStep * DT;
                if (rampedGain <= GAIN) { rampedGain = GAIN; initialising = false; }
            }
            batch_rg[base + i] = rampedGain;
        }

        // Phase 2: HW sections 5-8 (float, IP state feeds forward unnormalized)
        // Reset hw_q to current normalized q[] at start of each batch
        // (this matches the spec's AhrsHW_ProcessBatch initialising from ahrs->quaternion)
        hw_q[0] = q[0];  hw_q[1] = q[1];  hw_q[2] = q[2];  hw_q[3] = q[3];

        static Quat hw_out[BATCH_SIZE];
        for (int i = 0; i < BATCH_SIZE; i++)
            hw_out[i] = hw_step(sensor_data[base + i], hgX, hgY, hgZ, batch_rg[base + i]);

        // Phase 3: normalize each HW output, update q[] sequentially
        for (int i = 0; i < BATCH_SIZE; i++)
            float_q[base + i] = sw_normalize_and_update(hw_out[i]);
    }

    // ── 3. Write float reference to CSV ─────────────────────────────────────
    FILE *ff = fopen(FLOAT_CSV, "w");
    if (ff) {
        fprintf(ff, "sample,qw,qx,qy,qz\n");
        for (int i = 0; i < n_loaded; i++)
            fprintf(ff, "%d,%.8f,%.8f,%.8f,%.8f\n",
                    i, float_q[i].w, float_q[i].x, float_q[i].y, float_q[i].z);
        fclose(ff);
        printf("Float reference written to %s\n", FLOAT_CSV);
    }

    // ── 4. Load VP output (produced by ./ahrs_vp) ───────────────────────────
    static Quat vp_q[N_SAMPLES];
    int n_vp = 0;

    FILE *fv = fopen(VP_CSV, "r");
    if (!fv) {
        fprintf(stderr,
            "\nWARNING: %s not found.\n"
            "  Run './ahrs_vp' first to generate it, then re-run './compare'.\n"
            "  Float reference has been written to %s.\n\n",
            VP_CSV, FLOAT_CSV);
        return 0;
    }
    (void)fgets(line, sizeof(line), fv);  // skip header
    while (fgets(line, sizeof(line), fv) && n_vp < N_SAMPLES) {
        int idx;
        float qw, qx, qy, qz;
        if (sscanf(line, "%d,%f,%f,%f,%f", &idx, &qw, &qx, &qy, &qz) == 5)
            vp_q[n_vp++] = {qw, qx, qy, qz};
    }
    fclose(fv);
    printf("Loaded %d VP quaternions from %s\n\n", n_vp, VP_CSV);

    // ── 5. Compare ───────────────────────────────────────────────────────────
    FILE *fc = fopen(CMP_CSV, "w");
    if (fc)
        fprintf(fc, "sample,"
                    "float_qw,float_qx,float_qy,float_qz,"
                    "vp_qw,vp_qx,vp_qy,vp_qz,"
                    "q_l2_error,"
                    "float_roll,float_pitch,float_yaw,"
                    "vp_roll,vp_pitch,vp_yaw,"
                    "err_roll,err_pitch,err_yaw\n");

    float max_q_err   = 0.f, sum_q_err   = 0.f;
    float max_roll_e  = 0.f, sum_roll_e  = 0.f;
    float max_pitch_e = 0.f, sum_pitch_e = 0.f;
    float max_yaw_e   = 0.f, sum_yaw_e   = 0.f;
    int   n_cmp = (n_loaded < n_vp) ? n_loaded : n_vp;

    printf("%-6s  %12s%12s%12s  |  %12s%12s%12s  |  %9s%9s%9s\n",
           "Sample",
           "Roll_F", "Pitch_F", "Yaw_F",
           "Roll_VP", "Pitch_VP", "Yaw_VP",
           "dRoll", "dPitch", "dYaw");
    printf("------------------------------------"
           "------------------------------------"
           "---------------------\n");

    for (int i = 0; i < n_cmp; i++) {
        // Quaternion L2 error
        float dw = float_q[i].w - vp_q[i].w;
        float dx = float_q[i].x - vp_q[i].x;
        float dy = float_q[i].y - vp_q[i].y;
        float dz = float_q[i].z - vp_q[i].z;
        float q_err = sqrtf(dw*dw + dx*dx + dy*dy + dz*dz);
        sum_q_err += q_err;
        if (q_err > max_q_err) max_q_err = q_err;

        // Euler angle errors
        float fr, fp, fy, vr, vp, vy;
        quat_to_euler(float_q[i], fr, fp, fy);
        quat_to_euler(vp_q[i],    vr, vp, vy);

        float er = fabsf(fr - vr);
        float ep = fabsf(fp - vp);
        float ey = fabsf(fy - vy);
        // Wrap to [0, 180]
        if (er > 180.f) er = 360.f - er;
        if (ep > 180.f) ep = 360.f - ep;
        if (ey > 180.f) ey = 360.f - ey;

        sum_roll_e  += er;  if (er > max_roll_e)  max_roll_e  = er;
        sum_pitch_e += ep;  if (ep > max_pitch_e) max_pitch_e = ep;
        sum_yaw_e   += ey;  if (ey > max_yaw_e)   max_yaw_e   = ey;

        // Print every 10th sample (one per batch) to keep output readable
        if (i % 10 == 9) {
            printf("%-6d  %+12.3f%+12.3f%+12.3f  |  %+12.3f%+12.3f%+12.3f  |  %9.4f%9.4f%9.4f\n",
                   i, fr, fp, fy, vr, vp, vy, er, ep, ey);
        }

        if (fc)
            fprintf(fc, "%d,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,"
                        "%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    i,
                    float_q[i].w, float_q[i].x, float_q[i].y, float_q[i].z,
                    vp_q[i].w,    vp_q[i].x,    vp_q[i].y,    vp_q[i].z,
                    q_err,
                    fr, fp, fy, vr, vp, vy, er, ep, ey);
    }
    if (fc) { fclose(fc); printf("\nDetailed comparison written to %s\n", CMP_CSV); }

    // ── 6. Summary ───────────────────────────────────────────────────────────
    printf("\n");
    printf("=============================================================\n");
    printf("  Summary  (float ref vs VP sc_fixed,  %d samples)\n", n_cmp);
    printf("=============================================================\n");
    printf("  Quaternion L2 error:   mean = %.6f   max = %.6f\n",
           sum_q_err / n_cmp, max_q_err);
    printf("  Roll  error (°):       mean = %.4f    max = %.4f\n",
           sum_roll_e / n_cmp, max_roll_e);
    printf("  Pitch error (°):       mean = %.4f    max = %.4f\n",
           sum_pitch_e / n_cmp, max_pitch_e);
    printf("  Yaw   error (°):       mean = %.4f    max = %.4f\n",
           sum_yaw_e / n_cmp, max_yaw_e);

    // Final orientation
    float fr, fp, fy, vr, vp, vy;
    quat_to_euler(float_q[n_cmp-1], fr, fp, fy);
    quat_to_euler(vp_q[n_cmp-1],    vr, vp, vy);
    printf("\n  Final orientation (sample %d):\n", n_cmp-1);
    printf("    %10s  %10s  %10s\n", "Roll°", "Pitch°", "Yaw°");
    printf("    %+10.3f  %+10.3f  %+10.3f   (float ref)\n", fr, fp, fy);
    printf("    %+10.3f  %+10.3f  %+10.3f   (VP sc_fixed)\n", vr, vp, vy);
    printf("    %+10.3f  %+10.3f  %+10.3f   (error)\n",
           fr-vr, fp-vp, fy-vy);
    printf("=============================================================\n\n");

    return 0;
}


