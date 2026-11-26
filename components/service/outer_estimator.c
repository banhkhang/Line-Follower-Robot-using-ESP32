/*
 * ============================================================================
 * outer_estimator.c - Trien khai Bo loc Alpha-Beta, Tach nhanh Y Gauss-Newton
 *                     va Bo quan sat Luenberger thich nghi
 * ============================================================================
 * Tham chieu: SMC_controller14.m (lines 58-82, 249-282, 350-379, 616-698)
 *             va robot_params.h (B = 0.180 m, d1 = 0.055 m).
 */

#include "outer_estimator.h"
#include "robot_params.h"
#include <math.h>
#include <stddef.h>

/*
 * Toa do vat ly cac mat cam bien S1..S7 trong HE TRUC SMC (+Y = ben TRAI):
 *  - Tren PCB that: S1 nam ben TRAI (-27 mm trong sensor_service), S7 ben PHAI (+27 mm).
 *  - Trong he truc xe chuan ISO/SMC: ben TRAI la +Y -> dao dau thanh [+27 .. -27] mm!
 */
static const float s_y_sens_smc_m[LINE_SENSOR_MAX_COUNT] = {
    +0.027f,    /* [0] S1 (Trai ngoai cung) */
    +0.018f,    /* [1] S2 */
    +0.009f,    /* [2] S3 */
     0.000f,    /* [3] S4 (Tam giua) */
    -0.009f,    /* [4] S5 */
    -0.018f,    /* [5] S6 */
    -0.027f     /* [6] S7 (Phai ngoai cung) */
};

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/*
 * Xap xi Pade [3/2] cho ham tanh(u):
 *   tanh(u) ≈ u*(27 + u^2) / (27 + 9*u^2) voi |u| < 3, va ±1 khi |u| >= 3.
 * Sai so cuc dai < 0.005, nhanh hon tanhf() ~5x tren ESP32 (khong can ham mu expf).
 */
static inline float fast_tanh(float u)
{
    if (u >= 3.0f)  return  1.0f;
    if (u <= -3.0f) return -1.0f;
    float u2 = u * u;
    return u * (27.0f + u2) / (27.0f + 9.0f * u2);
}

void outer_estimator_reset(outer_estimator_t *est, float e2_init_m,
                           float ts_sens_s, float ts_in_s)
{
    if (est == NULL) return;

    /*
     * Tinh he so bo loc Alpha-Beta tu tan so tu nhien wn_f va he so tat dan zeta_f:
     *   alpha = 1 - exp(-2 * zeta_f * wn_f * Ts)
     *   beta  = (wn_f * Ts)^2
     * Bo loc bac 2 nay loc tot nhieu tan so cao nhung van bam khong tre pha khi e2
     * bien thien tuyen tinh (robot di cheo qua line).
     */
    est->alpha_sens  = 1.0f - expf(-2.0f * OUTER_AB_ZETA * OUTER_AB_WN_RADS * ts_sens_s);
    est->beta_sens   = (OUTER_AB_WN_RADS * ts_sens_s) * (OUTER_AB_WN_RADS * ts_sens_s);

    est->alpha_in    = 1.0f - expf(-2.0f * OUTER_AB_ZETA * OUTER_AB_WN_RADS * ts_in_s);
    est->beta_in     = (OUTER_AB_WN_RADS * ts_in_s) * (OUTER_AB_WN_RADS * ts_in_s);

    est->e2_filt     = e2_init_m;
    est->de2_filt    = 0.0f;
    est->e2_hat      = e2_init_m;
    est->e3_hat      = 0.0f;
    est->c_hat       = 0.0f;
    est->y_gn_active = false;
    est->last_e2_raw = e2_init_m;
}

void outer_estimator_reset_observer(outer_estimator_t *est, float e2_init_m)
{
    if (est == NULL) return;
    est->e2_hat      = e2_init_m;
    est->e3_hat      = 0.0f;
    est->c_hat       = 0.0f;
    est->y_gn_active = false;
}

void outer_estimator_step_200hz(outer_estimator_t *est, float e2_raw_m,
                                bool line_ok, bool in_y_zone, float ts_sens_s)
{
    if (est == NULL) return;

    /* Buoc du doan tuyen tinh (Predict) */
    float e2_pred = est->e2_filt + est->de2_filt * ts_sens_s;

    if (!in_y_zone) {
        est->y_gn_active = false;
        if (line_ok) {
            /* Duong don binh thuong: cap nhat Alpha-Beta o 200 Hz */
            float res     = e2_raw_m - e2_pred;
            est->e2_filt  = e2_pred + est->alpha_sens * res;
            est->de2_filt = est->de2_filt + (est->beta_sens / ts_sens_s) * res;
            est->last_e2_raw = e2_raw_m;
        } else {
            /* Mat line tam thoi: giu du doan co suy giam van toc ngang de tranh troi xa */
            est->e2_filt   = clampf(e2_pred, -0.035f, 0.035f);
            est->de2_filt *= 0.90f;
        }
    } else {
        /*
         * Trong cua so nga ba chu Y:
         * Khong dung trong tam duong don o 200 Hz (se bi keo vao khe trang giua 2 nhanh).
         * Chi tich phan du doan tuyen tinh, cho Gauss-Newton cap nhat o nhip 100 Hz.
         */
        est->e2_filt = clampf(e2_pred, -0.035f, 0.035f);
    }
}

void outer_estimator_step_y_branch_100hz(outer_estimator_t *est,
                                         const float i_eff[LINE_SENSOR_MAX_COUNT],
                                         float total_i,
                                         tcrt_profile_t profile,
                                         uint8_t load_kg,
                                         float ts_in_s)
{
    if (est == NULL || i_eff == NULL) return;

    est->y_gn_active = false;
    if (total_i < OUTER_LINE_TOTAL_I_MIN) {
        return;
    }

    const int k_start = (profile == TCRT_PROFILE_7S) ? 0 : 1;
    const int k_end   = (profile == TCRT_PROFILE_7S) ? 6 : 5;
    const float single_line_tot =
        (profile == TCRT_PROFILE_7S) ? OUTER_SINGLE_LINE_TOT_7S : OUTER_SINGLE_LINE_TOT_5S;

    float e_pred = est->e2_filt;

    /*
     * Buoc 1: Dat cua so Gating quang hoc quanh vi tri du doan e_pred (+-21 mm)
     * de loai bo nhanh khong lien quan khi 2 nhanh da tach xa nhau.
     */
    float w_gate[LINE_SENSOR_MAX_COUNT]  = {0};
    float i_gated[LINE_SENSOR_MAX_COUNT] = {0};
    float tot_g = 0.0f;

    for (int k = k_start; k <= k_end; k++) {
        if (fabsf(s_y_sens_smc_m[k] - e_pred) <= OUTER_GN_GATE_HALF_W_M) {
            w_gate[k]  = 1.0f;
            i_gated[k] = i_eff[k];
            tot_g     += i_eff[k];
        }
    }

    /* Neu cua so gating khong bat duoc line -> mo rong lai toan bo mat dang hoat dong */
    if (tot_g < OUTER_LINE_TOTAL_I_MIN) {
        tot_g = 0.0f;
        for (int k = k_start; k <= k_end; k++) {
            w_gate[k]  = 1.0f;
            i_gated[k] = i_eff[k];
            tot_g     += i_eff[k];
        }
    }

    if (tot_g < OUTER_LINE_TOTAL_I_MIN) {
        return;
    }

    /* Tinh trong tam trong cua so gating */
    float w_sum = 0.0f;
    for (int k = k_start; k <= k_end; k++) {
        w_sum += s_y_sens_smc_m[k] * i_gated[k];
    }
    float e_cent = w_sum / tot_g;
    float e_lat  = e_cent;

    /*
     * Buoc 2: Neu tong cuong do > single_line_tot -> ca 2 nhanh Y dang nam trong
     * cua so gating -> kich hoat khop mau Gauss-Newton (3 vong lap) de khoa vao
     * mep ngoai sach cua nhanh muc tieu:
     *   - load_kg == 1 (1 kg) -> nhanh TRAI (+Y trong he SMC) => branch_sign = +1.0
     *   - load_kg == 2 (2 kg) -> nhanh PHAI (-Y trong he SMC) => branch_sign = -1.0
     */
    if (tot_g > single_line_tot) {
        est->y_gn_active = true;

        const float inv_delta_opt = 1.0f / OUTER_OPT_DELTA_M;
        const float line_half_w   = ROBOT_LINE_WIDTH_M * 0.5f;
        const float branch_sign   = (load_kg == 1U) ? +1.0f : -1.0f;

        float e = e_pred;
        for (int iter = 0; iter < OUTER_GN_ITERS; iter++) {
            float num   = 0.0f;
            float denom = 1e-6f;

            for (int k = k_start; k <= k_end; k++) {
                if (w_gate[k] <= 0.0f) continue;

                float diff  = s_y_sens_smc_m[k] - e;
                float dist  = fabsf(diff);
                float u     = (line_half_w - dist) * inv_delta_opt;
                float th    = fast_tanh(u);
                float i_lin = 0.5f * (th + 1.0f);

                /* Mo hinh quang hoc 1 vach don 26 mm & dao ham dT/de */
                float t0 = i_lin - OUTER_GN_NOISE_OFFSET;
                float dt = 0.0f;
                if (t0 < OUTER_GN_MIN_ACTIVE) {
                    t0 = 0.0f;
                    dt = 0.0f;
                } else {
                    float sgn_diff = (diff >= 0.0f) ? 1.0f : -1.0f;
                    dt = 0.5f * (1.0f - th * th) * sgn_diff * inv_delta_opt;
                }

                float res = i_gated[k] - t0;

                /*
                 * Danh trong so bat doi xung (Asymmetric IRLS Weighting):
                 * Cac mat nam phia nhanh khong mong muon (side_pos < 0) co cuong do
                 * cao hon mau 1 vach (res > 0) bi triet trong so (w = 0 hoac 0.15),
                 * buoc Gauss-Newton chi khoa chat vao mep ngoai cua nhanh muc tieu.
                 */
                float w = w_gate[k];
                float side_pos = branch_sign * diff;
                if (side_pos < -0.006f && res > 0.03f) {
                    w = 0.0f;
                } else if (side_pos < 0.002f && res > 0.04f) {
                    w = w * 0.15f;
                }

                num   += w * dt * res;
                denom += w * dt * dt;
            }

            float step_e = clampf(num / denom, -OUTER_GN_MAX_STEP_M, OUTER_GN_MAX_STEP_M);
            e += step_e;
        }

        /* Tron muot giua centroid va Gauss-Newton de tranh buoc nhay dot ngot */
        float alpha_blend = clampf((tot_g - single_line_tot) / OUTER_GN_BLEND_SPAN, 0.0f, 1.0f);
        e_lat = (1.0f - alpha_blend) * e_cent + alpha_blend * e;
        e_lat = clampf(e_lat, -0.030f, 0.030f);
    }

    /* Buoc 3: Cap nhat bo loc Alpha-Beta theo nhip 100 Hz */
    float res_in  = e_lat - e_pred;
    est->e2_filt  = e_pred + est->alpha_in * res_in;
    est->de2_filt = est->de2_filt + (est->beta_in / ts_in_s) * res_in;
    est->last_e2_raw = e_lat;
}

void outer_estimator_step_observer_100hz(outer_estimator_t *est,
                                         float v_act_mps,
                                         float w_act_rads,
                                         float v_cmd_mps,
                                         bool active_tracking,
                                         float ts_in_s)
{
    if (est == NULL) return;

    float v_obs = (fabsf(v_act_mps) > OUTER_OBS_V_ACTIVE_THR) ? v_act_mps : v_cmd_mps;

    if (!active_tracking || fabsf(v_obs) <= OUTER_OBS_V_ACTIVE_THR) {
        /* Khi dung hoac cho tai: giu e2_hat dong bo voi cam bien, khong cap nhat e3 */
        est->e2_hat = est->e2_filt;
        return;
    }

    float v_denom = fmaxf(fabsf(v_obs), OUTER_OBS_V_MIN_DENOM);

    /*
     * 1. Uoc luong do cong quy dao c_hat = w_act / v_denom qua bo loc thong thap bac 1
     *    (tau_c = 35 ms): giup SMC tinh thanh phan bu truyen thang do cong u_eq.
     */
    float c_raw   = w_act_rads / v_denom;
    float alpha_c = ts_in_s / (OUTER_OBS_TAU_C_S + ts_in_s);
    est->c_hat    = (1.0f - alpha_c) * est->c_hat + alpha_c * c_raw;

    /*
     * 2. Chon cap cuc Luenberger thich nghi theo sai lech du doan |err_obs|:
     *    - Khi bam em (|err_obs| nho): dung cuc cham (0.89, 0.92) -> loc nhieu cam bien tot.
     *    - Khi vao cua dot ngot (|err_obs| lon): chuyen muot sang cuc nhanh (0.75, 0.80)
     *      -> e3_hat bat kip goc tiep tuyen nhanh ma khong bi tre pha.
     */
    float err_obs = est->e2_filt - est->e2_hat;
    float blend   = clampf((fabsf(err_obs) - OUTER_OBS_ERR_BLEND_LO_M) / OUTER_OBS_ERR_BLEND_SPAN_M,
                           0.0f, 1.0f);
    float p1_obs  = (1.0f - blend) * OUTER_OBS_P1_SLOW + blend * OUTER_OBS_P1_FAST;
    float p2_obs  = (1.0f - blend) * OUTER_OBS_P2_SLOW + blend * OUTER_OBS_P2_FAST;

    /* Dat cuc roi rac (Ackermann / Pole Placement) cho cap (e2, e3) */
    float ld1 = 2.0f - (p1_obs + p2_obs);
    float ld2 = (p1_obs * p2_obs - 1.0f + ld1) / (v_denom * ts_in_s);

    /*
     * 3. Mo hinh dong hoc roi rac hoa Taylor bac 2 (theo PT (5) & SMC_controller14.m):
     *      e2_dot = v * sin(e3) - d1 * w
     *      e3_dot = c * v - w
     */
    float w_net       = w_act_rads - est->c_hat * v_obs;
    float e2_pred_obs = est->e2_hat
                      + v_obs * ts_in_s * sinf(est->e3_hat)
                      - ROBOT_SENSOR_OFFSET_M * ts_in_s * w_act_rads
                      - 0.5f * v_obs * (ts_in_s * ts_in_s) * w_net;
    float e3_pred_obs = est->e3_hat + ts_in_s * (est->c_hat * v_obs - w_act_rads);

    /* 4. Hieu chinh trang thai (Correction) */
    est->e2_hat = e2_pred_obs + ld1 * err_obs;
    est->e3_hat = clampf(e3_pred_obs + ld2 * err_obs,
                         -OUTER_OBS_E3_CLAMP_RAD, OUTER_OBS_E3_CLAMP_RAD);
}
