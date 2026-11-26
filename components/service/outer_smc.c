/*
 * ============================================================================
 * outer_smc.c - Trien khai Integral SMC, Bo lap lich thich nghi & Quy hoach toc do
 * ============================================================================
 */

#include "outer_smc.h"
#include "robot_params.h"
#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void outer_smc_get_default_params(outer_smc_params_t *out)
{
    if (out == NULL) return;
    out->v_req           = ROBOT_V_NOM_MPS;
    out->beta_smc        = OUTER_SMC_BETA_DEFAULT;
    out->k_smc           = OUTER_SMC_K_DEFAULT;
    out->phi_bl          = OUTER_SMC_PHI_BL_DEFAULT;
    out->ki_smc          = OUTER_SMC_KI_DEFAULT;
    out->z_max           = OUTER_SMC_Z_MAX_DEFAULT;
    out->w_max           = ROBOT_W_MAX_RADS;

    out->ts_min_s        = OUTER_SMC_TS_MIN_S;
    out->ts_max_s        = OUTER_SMC_TS_MAX_S;
    out->settle_margin_s = OUTER_SMC_SETTLE_MARGIN_S;
    out->settle_band_mps = OUTER_SMC_SETTLE_BAND_MPS;

    out->station_trim_m  = MARKER_AXLE_TRIM_M;
    out->goal_trim_m     = GOAL_AXLE_TRIM_M;
}

void outer_smc_set_params(outer_smc_t *smc, const outer_smc_params_t *params)
{
    if (smc == NULL || params == NULL) return;

    outer_smc_params_t p = *params;
    /* Kep an toan moi tham so nhan tu MATLAB de tranh gia tri vo ly gay mat on dinh */
    p.v_req           = clampf(p.v_req,           ROBOT_V_MIN_MPS, 0.70f);
    p.beta_smc        = clampf(p.beta_smc,        0.001f,          0.100f);
    p.k_smc           = clampf(p.k_smc,           0.01f,           1.50f);
    p.phi_bl          = clampf(p.phi_bl,          0.003f,          0.080f);
    p.ki_smc          = clampf(p.ki_smc,          0.0f,            8.0f);
    p.z_max           = clampf(p.z_max,           0.0f,            0.050f);
    p.w_max           = clampf(p.w_max,           0.5f,            ROBOT_W_MAX_RADS);

    /* Rang buoc chu ky vong ngoai trong [0.050 s .. 0.195 s] theo yeu cau thiet ke */
    p.ts_min_s        = clampf(p.ts_min_s,        0.050f,          0.180f);
    p.ts_max_s        = clampf(p.ts_max_s,        p.ts_min_s,      0.195f);
    p.settle_margin_s = clampf(p.settle_margin_s, 0.000f,          0.080f);
    p.settle_band_mps = clampf(p.settle_band_mps, 0.008f,          0.080f);

    p.station_trim_m  = clampf(p.station_trim_m, -0.050f,         +0.050f);
    p.goal_trim_m     = clampf(p.goal_trim_m,    -0.050f,         +0.050f);

    smc->p = p;
}

void outer_smc_reset_state(outer_smc_t *smc)
{
    if (smc == NULL) return;
    smc->z_int            = 0.0f;
    smc->sigma            = 0.0f;
    smc->sat_sigma        = 0.0f;
    smc->w_cmd            = 0.0f;
    smc->t_since_update_s = smc->p.ts_min_s; /* San sang tinh lenh ngay o nhip dau tien */
    smc->settle_dwell_s   = 0.0f;
    smc->last_dt_smc_s    = smc->p.ts_min_s;
}

void outer_smc_init(outer_smc_t *smc, const outer_smc_params_t *params)
{
    if (smc == NULL) return;
    if (params != NULL) {
        outer_smc_set_params(smc, params);
    } else {
        outer_smc_params_t def;
        outer_smc_get_default_params(&def);
        outer_smc_set_params(smc, &def);
    }
    outer_smc_reset_state(smc);
}

bool outer_smc_step_adaptive(outer_smc_t *smc,
                             float e2_m,
                             float e3_hat_rad,
                             float c_hat,
                             float v_cmd_mps,
                             float seg_elapsed_s,
                             float va_ref_mps,
                             float va_act_mps,
                             float vb_ref_mps,
                             float vb_act_mps,
                             bool active_tracking,
                             float dt_tick_s)
{
    if (smc == NULL) return false;

    if (!active_tracking) {
        smc->w_cmd            = 0.0f;
        smc->sigma            = 0.0f;
        smc->sat_sigma        = 0.0f;
        smc->t_since_update_s = smc->p.ts_min_s;
        smc->settle_dwell_s   = 0.0f;
        return false;
    }

    smc->t_since_update_s += dt_tick_s;

    /*
     * Kiem tra trang thai xac lap cua 2 banh vong trong:
     * Dai xac lap cho phep = max(settle_band_mps, 5% van toc dat) de nhat quan voi
     * chi tieu Ts,5% trong Bang 6.1 cua bao cao vong trong.
     */
    float band_a = fmaxf(smc->p.settle_band_mps, 0.05f * fabsf(va_ref_mps));
    float band_b = fmaxf(smc->p.settle_band_mps, 0.05f * fabsf(vb_ref_mps));
    bool  wheels_settled = (fabsf(va_ref_mps - va_act_mps) <= band_a) &&
                           (fabsf(vb_ref_mps - vb_act_mps) <= band_b);

    if (wheels_settled) {
        smc->settle_dwell_s += dt_tick_s;
    } else {
        smc->settle_dwell_s = 0.0f;
    }

    /*
     * Dieu kien phat lenh SMC moi:
     *  - Da qua chu ky toi thieu ts_min_s (>= 50 ms) VA vong trong da xac lap du
     *    le thoi gian settle_margin_s (mac dinh 20 ms), HOAC
     *  - Da cham chu ky toi da ts_max_s (< 200 ms, bao dam gioi han tre pha).
     */
    bool settled_ready = (smc->t_since_update_s >= smc->p.ts_min_s) &&
                         (smc->settle_dwell_s >= smc->p.settle_margin_s);
    bool timeout_ready = (smc->t_since_update_s >= smc->p.ts_max_s);

    if (!settled_ready && !timeout_ready) {
        return false; /* Giu nguyen w_cmd (Zero-Order Hold) */
    }

    float dt_smc = smc->t_since_update_s;
    smc->last_dt_smc_s    = dt_smc;
    smc->t_since_update_s = 0.0f;
    smc->settle_dwell_s   = 0.0f;

    /*
     * --- LUAT DIEU KHIEN TRUOT TICH PHAN (Doria-Cerezo et al., ECC 2019) ---
     * 1) Khau tich phan z_int (chi bat sau 0.30 s khoi hanh de tranh windup luc khoi dong)
     *    su dung chinh xac chu ky thuc dt_smc.
     */
    if (seg_elapsed_s > OUTER_SMC_INT_DELAY_S) {
        smc->z_int = clampf(smc->z_int + smc->p.ki_smc * e2_m * dt_smc,
                            -smc->p.z_max, smc->p.z_max);
    }

    /* 2) Mat truot sigma va ham bao hoa lop bien sat(sigma / phi_bl) chong rung */
    smc->sigma     = e2_m + smc->p.beta_smc * e3_hat_rad + smc->z_int;
    smc->sat_sigma = clampf(smc->sigma / smc->p.phi_bl, -1.0f, 1.0f);

    /* 3) Dieu khien tuong duong u_eq + thanh phan chuyen mach -> van toc goc dat w_cmd */
    float denom_smc = ROBOT_SENSOR_OFFSET_M + smc->p.beta_smc;
    float u_eq      = (v_cmd_mps * (sinf(e3_hat_rad) + smc->p.beta_smc * c_hat)
                       + smc->p.ki_smc * e2_m) / denom_smc;
    float w_raw     = u_eq + (smc->p.k_smc / denom_smc) * smc->sat_sigma;

    smc->w_cmd = clampf(w_raw, -smc->p.w_max, smc->p.w_max);
    return true;
}

float outer_calc_decel_velocity(float d_rem_m, float v_req_mps, float v_min_mps,
                                float d_dec_m, float d_margin_m)
{
    if (v_req_mps <= v_min_mps) {
        return v_min_mps;
    }
    if (d_rem_m >= d_dec_m) {
        return v_req_mps;
    }
    if (d_rem_m <= d_margin_m) {
        return v_min_mps;
    }

    float dec_span = d_dec_m - d_margin_m;
    if (dec_span < 0.02f) dec_span = 0.02f;

    /* Gia toc giam toc khong doi a_dec = (v_req^2 - v_min^2) / (2 * dec_span) */
    float a_dec = (v_req_mps * v_req_mps - v_min_mps * v_min_mps) / (2.0f * dec_span);
    float v_sq  = v_min_mps * v_min_mps + 2.0f * a_dec * (d_rem_m - d_margin_m);
    float v_out = sqrtf(fmaxf(v_sq, 0.0f));
    return clampf(v_out, v_min_mps, v_req_mps);
}

float outer_calc_y_junction_speed(float s_enc_m, float v_req_mps)
{
    if (s_enc_m <= TRACK_Y_DIP_START_M || s_enc_m >= TRACK_Y_DIP_END_M) {
        return v_req_mps;
    }
    float u = (s_enc_m - TRACK_Y_DIP_START_M) / (TRACK_Y_DIP_END_M - TRACK_Y_DIP_START_M);
    float v_junc = v_req_mps - TRACK_Y_DIP_DELTA_V_MPS * 0.5f * (1.0f - cosf(2.0f * (float)M_PI * u));
    return fmaxf(v_junc, ROBOT_V_MIN_MPS);
}

float outer_estimate_brake_lead_m(float v_mps, float m_load_kg)
{
    if (v_mps <= 0.0f) return 0.0f;

    /*
     * Noi suy tuyen tinh quang duong phanh Short Brake o v_ref = 0.70 m/s tu Bang 6.6:
     *   0 kg -> 64 mm, 1 kg -> 71 mm, 2 kg -> 78 mm (tang 7 mm/kg do tang quan tinh & CoG)
     * Sau do quy doi theo binh phuong van toc (dong nang ~ v^2) + tre 1 chu ky 5 ms.
     */
    float m_clamped = clampf(m_load_kg, 0.0f, 2.5f);
    float d_stop_07 = ROBOT_STOP_DIST_0KG_M + 0.007f * m_clamped;
    float v_ratio   = v_mps / ROBOT_STOP_DIST_REF_V_MPS;
    float d_brake   = d_stop_07 * (v_ratio * v_ratio) + v_mps * INNER_LOOP_TS_S;
    return clampf(d_brake, 0.0f, 0.030f);
}
