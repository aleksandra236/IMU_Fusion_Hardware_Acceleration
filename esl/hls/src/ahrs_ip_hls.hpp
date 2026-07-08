#ifndef AHRS_IP_HLS_HPP
#define AHRS_IP_HLS_HPP

#include <ap_fixed.h>
#include <stdint.h>

// ============================================================================
// Fixed-point tipovi – V3 Final, originalne bit-sirine iz bitske analize
// ============================================================================

typedef ap_fixed <18, 2, AP_RND, AP_SAT>   quat_t;      // kvaternion    [-2, 2)
typedef ap_fixed <26, 1, AP_TRN, AP_WRAP>  dot_t;       // skalarni pro. [-1, 1)
typedef ap_fixed <26, 2, AP_TRN, AP_WRAP>  norm_t;      // normirani vek [-2, 2)
typedef ap_fixed <20, 3, AP_TRN, AP_WRAP>  accel_t;     // akcelerometar [-4, 4)
typedef ap_fixed <26, 2, AP_TRN, AP_WRAP>  hg_t;        // halfGravity   [-2, 2)
typedef ap_ufixed<22, 2, AP_TRN, AP_WRAP>  magsq_t;     // magnituda^2   [0, 4)
typedef ap_ufixed<24, 6, AP_TRN, AP_WRAP>  invmag_t;    // inv-mag       [0, 64)
typedef ap_fixed <22, 2, AP_TRN, AP_WRAP>  halfgyro_t;  // polu-ziro     [-2, 2)
typedef ap_ufixed<20, 4, AP_TRN, AP_WRAP>  gain_t;      // pojacanje     [0, 16)
typedef ap_fixed <26, 5, AP_TRN, AP_WRAP>  adjhg_t;     // adj polu-ziro [-16, 16)
typedef ap_fixed <27, 5, AP_TRN, AP_WRAP>  dq_t;        // delta-kvaternion
typedef ap_ufixed<20, 0, AP_TRN, AP_WRAP>  dt_t;        // deltaTime     [0, 1)

void ahrs_ip_hls(
    quat_t  qw_in,  quat_t  qx_in,  quat_t  qy_in,  quat_t  qz_in,
    float   gx,     float   gy,     float   gz,
    float   ax,     float   ay,     float   az,
    float   halfGravityX, float halfGravityY, float halfGravityZ,
    float   deltaTime,    float rampedGain,
    quat_t &qw_out, quat_t &qx_out, quat_t &qy_out, quat_t &qz_out
);

#endif
