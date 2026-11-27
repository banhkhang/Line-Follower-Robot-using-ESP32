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
    out->w_max           = OUTER_SMC_W_MAX_DEFAULT;

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
    smc->last_e2_m        = 0.0f;
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
                             bool line_ok,
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
     * Dai xac lap cho phep = max(settle_band_mps, 8% van toc dat) de phu hop voi
     * dong luc hoc banh that khi danh lai dong tren bat.
     */
    float band_a = fmaxf(smc->p.settle_band_mps, 0.08f * fabsf(va_ref_mps));
    float band_b = fmaxf(smc->p.settle_band_mps, 0.08f * fabsf(vb_ref_mps));
    bool  wheels_settled = (fabsf(va_ref_mps - va_act_mps) <= band_a) &&
                           (fabsf(vb_ref_mps - vb_act_mps) <= band_b);

    if (wheels_settled) {
        smc->settle_dwell_s += dt_tick_s;
    } else {
        smc->settle_dwell_s = 0.0f;
    }

    /*
     * Dieu kien phat lenh SMC moi (luon ton trong ts_min_s <= dt_smc <= ts_max_s):
     *  (1) Da qua ts_min_s VA vong trong da xac lap du settle_margin_s, HOAC
     *  (2) Early-release: da qua ts_min_s VA xe vua cat qua tam line (doi dau e2)
     *      hoac w_cmd dang lai nguoc chieu can sua -> cap nhat ngay, tranh giu lenh
     *      qua lau gay vang sang ben kia, HOAC
     *  (3) Da cham chu ky toi da ts_max_s (< 200 ms, bao dam gioi han tre pha).
     */
    bool min_elapsed   = (smc->t_since_update_s >= smc->p.ts_min_s);
    bool settled_ready = min_elapsed && (smc->settle_dwell_s >= smc->p.settle_margin_s);
    bool zero_crossed  = (smc->last_e2_m * e2_m < 0.0f) && (fabsf(e2_m) > 0.0005f);
    bool wrong_steer   = (e2_m * smc->w_cmd < 0.0f) &&
                         (fabsf(e2_m) > 0.0020f) && (fabsf(smc->w_cmd) > 0.30f);
    bool early_release = min_elapsed && (zero_crossed || wrong_steer);
    bool timeout_ready = (smc->t_since_update_s >= smc->p.ts_max_s);

    if (!settled_ready && !early_release && !timeout_ready) {
        return false; /* Giu nguyen w_cmd (Zero-Order Hold) */
    }

    float dt_smc = smc->t_since_update_s;
    smc->last_dt_smc_s    = dt_smc;
    smc->t_since_update_s = 0.0f;
    smc->settle_dwell_s   = 0.0f;
    smc->last_e2_m        = e2_m;

    /*
     * --- LUAT DIEU KHIEN TRUOT TICH PHAN (Doria-Cerezo et al., ECC 2019) ---
     * 1) Khau tich phan z_int: chi tich luy khi dang nhin thay line, da chay > 0.30s
     *    va v_cmd > 0.05 m/s (dong bang khi mat line de chong windup).
     */
    if (line_ok && seg_elapsed_s > OUTER_SMC_INT_DELAY_S && v_cmd_mps > 0.05f) {
        smc->z_int = clampf(smc->z_int + smc->p.ki_smc * e2_m * dt_smc,
                            -smc->p.z_max, smc->p.z_max);
    } else if (!line_ok) {
        smc->z_int *= 0.90f;
    }

    /* 2) Mat truot sigma va ham bao hoa lop bien sat(sigma / phi_bl) chong rung */
    smc->sigma     = e2_m + smc->p.beta_smc * e3_hat_rad + smc->z_int;
    smc->sat_sigma = clampf(smc->sigma / smc->p.phi_bl, -1.0f, 1.0f);

    /*
     * 3) Dieu khien tuong duong u_eq + thanh phan chuyen mach co ty le theo van toc
     *    khoi hanh (alpha_v): tranh tinh trang xe van quay tai cho khi S-curve moi
     *    tang toc tu v_cmd ~ 0 m/s!
     */
    float denom_smc = ROBOT_SENSOR_OFFSET_M + smc->p.beta_smc;
    float alpha_v   = clampf(v_cmd_mps / fmaxf(smc->p.v_req, 0.10f), 0.10f, 1.0f);
    float u_eq      = (v_cmd_mps * (sinf(e3_hat_rad) + smc->p.beta_smc * c_hat)
                       + alpha_v * smc->p.ki_smc * e2_m) / denom_smc;
    float w_raw     = u_eq + alpha_v * (smc->p.k_smc / denom_smc) * smc->sat_sigma;

    /*
     * Gioi han van toc goc dong + slew-rate thong minh:
     *  - Cung chieu lai: kep |dw| <= OUTER_SMC_DW_MAX_STEP de chong giat 100% PWM.
     *  - Dao chieu lai (w_target * w_cmd < 0): cho phep cat qua 0 ngay lap tuc roi moi
     *    kep bien do chieu moi (<= 0.7 * DW_MAX_STEP) de khong bi ket sai chieu them 60 ms!
     */
    float w_dyn_max = fminf(smc->p.w_max, fmaxf(0.40f, v_cmd_mps / 0.12f));
    float w_target  = clampf(w_raw, -w_dyn_max, w_dyn_max);
    if (w_target * smc->w_cmd < 0.0f) {
        float rev_cap = 0.70f * OUTER_SMC_DW_MAX_STEP;
        smc->w_cmd = clampf(w_target, -rev_cap, rev_cap);
    } else {
        float dw   = clampf(w_target - smc->w_cmd, -OUTER_SMC_DW_MAX_STEP, OUTER_SMC_DW_MAX_STEP);
        smc->w_cmd = clampf(smc->w_cmd + dw, -w_dyn_max, w_dyn_max);
    }
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
