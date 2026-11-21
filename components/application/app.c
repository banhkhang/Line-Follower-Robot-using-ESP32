#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include "app.h"
#include "ota.h"
#include "wifi_ap.h"
#include "config.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

static const char *TAG = "hil_app";

/* ============================================================================
 * 1. SENSOR, MOTOR, OBSERVER & SMC CONSTANTS
 * ============================================================================ */
static const float Y_SENS_M[7]      = {0.030f, 0.020f, 0.010f, 0.000f, -0.010f, -0.020f, -0.030f};
static const float OPT_SIGMA_M      = 0.006f;
static const float ADC_NOISE_FLOOR  = 0.085f;

/* Bench-identified JGB37-520 (333 RPM) 2-DOF PI gains */
static const float KFF_GAIN         = 86.23f;
static const float PWM_DEADZONE_0   = 2.10f;
static const float INT_BAND_BASE    = 0.035f;

/* Variable-mass coupling parameters */
static const float TAU_0_SEC        = 0.055f;
static const float GAMMA_INERTIA    = 0.0384f;
static const float K_PWM_ROLL       = 1.55f;

static const float PWM_MAX_LIMIT    = 100.0f;
static const float PWM_MIN_REVERSE  = -100.0f;

/* Alpha-Beta pre-filter parameters */
static const float WN_FILT          = 95.0f;
static const float ZETA_FILT        = 0.92f;

/* Adaptive Luenberger Observer poles (200 Hz) */
static const float P1_SLOW          = 0.9292f;
static const float P2_SLOW          = 0.9470f;
static const float P1_FAST          = 0.8272f;
static const float P2_FAST          = 0.8706f;
static const float TAU_CURV_SEC     = 0.045f;
static const float V_OBS_MIN_DENOM  = 0.25f;

/* ============================================================================
 * 2. SHARED HIL STATE & SYNCHRONIZATION
 * ============================================================================ */
typedef struct {
    uint16_t sensor_adc[7];
    uint8_t  mass_kg;
    float    vL_meas_ms;
    float    vR_meas_ms;
    uint32_t last_rx_ms;
    bool     link_active;
    float    pwmL_cmd;
    float    pwmR_cmd;
} shared_hil_state_t;

static shared_hil_state_t s_hil_state = { 0 };
static portMUX_TYPE       s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t       s_control_task_handle = NULL;

static inline uint32_t get_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ============================================================================
 * 3. GAUSS-NEWTON SINGLE-LINE TEMPLATE MATCHER (Y-JUNCTION)
 * ============================================================================ */
static inline float eval_single_line_template(float y_pos, float e_center)
{
    float dist = y_pos - e_center;
    float val  = expf(-(dist * dist) / (2.0f * OPT_SIGMA_M * OPT_SIGMA_M)) - ADC_NOISE_FLOOR;
    return (val > 0.0f) ? val : 0.0f;
}

static bool compute_lateral_error_gauss_newton(
    const uint16_t adc[7],
    uint8_t mass_kg,
    float e_pred_m,
    float *e_lat_out_m)
{
    float I_eff[7];
    float total_I = 0.0f;
    int num_clusters = 0;
    bool in_cluster = false;

    for (int i = 0; i < 7; i++) {
        float norm = (float)adc[i] / 1023.0f - ADC_NOISE_FLOOR;
        if (norm < 0.02f) norm = 0.0f;
        I_eff[i] = norm;
        total_I += norm;

        if (norm > 0.12f) {
            if (!in_cluster) {
                num_clusters++;
                in_cluster = true;
            }
        } else {
            in_cluster = false;
        }
    }

    if (total_I < 0.15f) {
        return false;
    }

    float weighted_sum = 0.0f;
    for (int i = 0; i < 7; i++) {
        weighted_sum += Y_SENS_M[i] * I_eff[i];
    }
    float e_centroid = weighted_sum / total_I;

    if (num_clusters < 2 && total_I <= 1.65f) {
        *e_lat_out_m = e_centroid;
        return true;
    }

    float branch_sign = (mass_kg == 1) ? 1.0f : -1.0f;
    float e_est = e_pred_m + branch_sign * 0.0025f;
    const float eps_e = 0.0001f;

    for (int iter = 0; iter < 3; iter++) {
        float sum_w_dT2    = 1e-6f;
        float sum_w_dT_res = 0.0f;

        for (int k = 0; k < 7; k++) {
            float T0 = eval_single_line_template(Y_SENS_M[k], e_est);
            float Tp = eval_single_line_template(Y_SENS_M[k], e_est + eps_e);
            float Tm = eval_single_line_template(Y_SENS_M[k], e_est - eps_e);
            float dT = (Tp - Tm) / (2.0f * eps_e);

            float res = I_eff[k] - T0;
            float w = 1.0f;

            float side_pos = branch_sign * (Y_SENS_M[k] - e_est);
            if (side_pos < -0.006f && res > 0.03f) {
                w = 0.0f;
            } else if (side_pos < 0.002f && res > 0.04f) {
                w = 0.15f;
            }

            sum_w_dT2    += w * dT * dT;
            sum_w_dT_res += w * dT * res;
        }

        float step_e = sum_w_dT_res / sum_w_dT2;
        if (step_e > 0.008f)  step_e = 0.008f;
        if (step_e < -0.008f) step_e = -0.008f;
        e_est += step_e;
    }

    if (e_est > 0.030f)  e_est = 0.030f;
    if (e_est < -0.030f) e_est = -0.030f;

    *e_lat_out_m = e_est;
    return true;
}

typedef enum {
    MISSION_TO_LOAD = 0,     /* Chạy từ điểm xuất phát tới trạm tải hàng */
    MISSION_WAIT_LOAD,       /* Dừng tại vạch ngang trạm tải hàng để nhận tải */
    MISSION_LEAVE_LOAD,      /* Chạy thẳng vượt qua vạch ngang trạm tải */
    MISSION_TO_GOAL,         /* Bám line + tách nhánh Y theo khối lượng tới đích */
    MISSION_STOPPED_GOAL,    /* Đã dừng hẳn tại vạch ngang đích */
    MISSION_STOPPED_LOST     /* Đã dừng hẳn do mất line liên tục >= 2 s */
} mission_state_t;

/* Phát hiện vạch ngang: >= CROSSBAR_MIN_ACTIVE mắt cảm biến cùng vượt ngưỡng */
static bool is_crossbar_detected(const uint16_t adc[7])
{
    int active_cnt = 0;
    for (int i = 0; i < 7; i++) {
        float norm = (float)adc[i] / 1023.0f - ADC_NOISE_FLOOR;
        if (norm >= CROSSBAR_SENSOR_TH) {
            active_cnt++;
        }
    }
    return (active_cnt >= CROSSBAR_MIN_ACTIVE);
}

/* ============================================================================
 * 4. ADAPTIVE LUENBERGER OBSERVER
 * ============================================================================ */
typedef struct {
    float e2_hat;
    float e3_hat;
    float c_hat;
} adaptive_observer_t;

static void observer_reset(adaptive_observer_t *obs, float e2_init)
{
    obs->e2_hat = e2_init;
    obs->e3_hat = 0.0f;
    obs->c_hat  = 0.0f;
}

static void observer_update(adaptive_observer_t *obs, float e2_meas, float v_act, float w_act)
{
    if (fabsf(v_act) < 0.04f) {
        obs->e2_hat = e2_meas;
        return;
    }

    float v_denom = fmaxf(fabsf(v_act), V_OBS_MIN_DENOM);

    float c_raw   = w_act / v_denom;
    float alpha_c = TS_SEC / (TAU_CURV_SEC + TS_SEC);
    obs->c_hat    = (1.0f - alpha_c) * obs->c_hat + alpha_c * c_raw;

    float err_obs = e2_meas - obs->e2_hat;
    float blend   = (fabsf(err_obs) - 0.0006f) / 0.0015f;
    if (blend < 0.0f) blend = 0.0f;
    if (blend > 1.0f) blend = 1.0f;

    float p1 = (1.0f - blend) * P1_SLOW + blend * P1_FAST;
    float p2 = (1.0f - blend) * P2_SLOW + blend * P2_FAST;

    float Ld1 = 2.0f - (p1 + p2);
    float Ld2 = (p1 * p2 - 1.0f + Ld1) / (v_denom * TS_SEC);

    float w_net   = w_act - obs->c_hat * v_act;
    float e2_pred = obs->e2_hat
                  + v_act * TS_SEC * sinf(obs->e3_hat)
                  - SENSOR_LOOKAHEAD_M * TS_SEC * w_act
                  - 0.5f * v_act * (TS_SEC * TS_SEC) * w_net;
    float e3_pred = obs->e3_hat - TS_SEC * w_net;

    obs->e2_hat = e2_pred + Ld1 * err_obs;
    obs->e3_hat = e3_pred + Ld2 * err_obs;

    if (obs->e3_hat > 1.05f)  obs->e3_hat = 1.05f;
    if (obs->e3_hat < -1.05f) obs->e3_hat = -1.05f;
}

/* ============================================================================
 * 5. MASS-ADAPTIVE 2-DOF PI MOTOR CONTROLLER
 * ============================================================================ */
typedef struct {
    float integral;
    float prev_err;
    float v_ref_shaped;
} mass_adaptive_pi_t;

static void motor_pi_reset(mass_adaptive_pi_t *pi)
{
    pi->integral     = 0.0f;
    pi->prev_err     = 0.0f;
    pi->v_ref_shaped = 0.0f;
}

static float motor_pi_update(mass_adaptive_pi_t *pi, float v_ref_target, float v_meas, float m_total)
{
    float tau_eff = TAU_0_SEC + GAMMA_INERTIA * m_total;
    float alpha_m = tau_eff / TAU_0_SEC;

    /* Khi chạy STEP TEST tắt slew-rate để đo chính xác đáp ứng bậc thang Ts, OS% */
#if HIL_RUN_MODE == MODE_INNER_STEP_TEST
    float v_ref = v_ref_target;
    pi->v_ref_shaped = v_ref_target;
#else
    float accel_max = 1.8f / sqrtf(alpha_m);
    float max_dv    = accel_max * TS_SEC;
    float dv        = v_ref_target - pi->v_ref_shaped;
    if (dv > max_dv)       dv = max_dv;
    else if (dv < -max_dv) dv = -max_dv;
    pi->v_ref_shaped += dv;
    float v_ref = pi->v_ref_shaped;
#endif

    float err = v_ref - v_meas;

    /* Khử cực đối tượng: zI = 1 / tau_eff => PM không đổi theo khối lượng */
    float kp_eff   = INNER_LOOP_WGC * tau_eff * KFF_GAIN;
    float ki_eff   = INNER_LOOP_WGC * KFF_GAIN;
    float int_band = INT_BAND_BASE * sqrtf(alpha_m);

    float pwm_comp = PWM_DEADZONE_0 + K_PWM_ROLL * m_total;
    float u_ff     = 0.0f;
    if (v_ref > 0.005f)       u_ff = KFF_GAIN * v_ref + pwm_comp;
    else if (v_ref < -0.005f) u_ff = KFF_GAIN * v_ref - pwm_comp;

    float u_p         = kp_eff * err;
    float pwm_min_dyn = (v_ref >= 0.0f) ? 0.0f : PWM_MIN_REVERSE;

    float i_candidate = pi->integral;
    if (fabsf(err) < int_band) {
        i_candidate += ki_eff * (0.5f * TS_SEC) * (err + pi->prev_err);
    }

    float u_unsat = u_ff + u_p + i_candidate;
    if (u_unsat > PWM_MAX_LIMIT) {
        if (err < 0.0f && fabsf(err) < int_band) pi->integral = i_candidate;
        u_unsat = PWM_MAX_LIMIT;
    } else if (u_unsat < pwm_min_dyn) {
        if (err > 0.0f && fabsf(err) < int_band) pi->integral = i_candidate;
        u_unsat = pwm_min_dyn;
    } else {
        pi->integral = i_candidate;
    }

    pi->prev_err = err;
    return u_unsat;
}

/* ============================================================================
 * 6. CORE 1 REAL-TIME CONTROL TASK (200 Hz via esp_timer)
 * ============================================================================ */
static void on_control_timer(void *arg)
{
    (void)arg;
    if (s_control_task_handle != NULL) {
        xTaskNotifyGive(s_control_task_handle);
    }
}

#if HIL_RUN_MODE == MODE_INNER_STEP_TEST
/* Quỹ đạo kiểm thử chuẩn 7.0 s:
 *  0.0 - 0.8 s: S-curve bậc 3 (0 -> 0.50 m/s)
 *  0.8 - 1.8 s: Giữ 0.50 m/s
 *  1.8 - 2.8 s: Bước nhảy lên 0.70 m/s
 *  2.8 - 3.8 s: Bước nhảy xuống 0.50 m/s
 *  3.8 - 4.8 s: Bước nhảy xuống 0.30 m/s
 *  4.8 - 5.8 s: Bước nhảy lên 0.50 m/s
 *  5.8 - 7.0 s: Phanh về 0.00 m/s (kiểm tra dừng)
 */
static float eval_inner_step_profile(float t_sec)
{
    if (t_sec < 0.8f) {
        float tau = t_sec / 0.8f;
        return 0.50f * (3.0f * tau * tau - 2.0f * tau * tau * tau);
    } else if (t_sec < 1.8f) {
        return 0.50f;
    } else if (t_sec < 2.8f) {
        return 0.70f;
    } else if (t_sec < 3.8f) {
        return 0.50f;
    } else if (t_sec < 4.8f) {
        return 0.30f;
    } else if (t_sec < 5.8f) {
        return 0.50f;
    } else {
        return 0.00f;
    }
}
#endif

typedef struct {
    adaptive_observer_t observer;
    mass_adaptive_pi_t  motor_ctrl_L;
    mass_adaptive_pi_t  motor_ctrl_R;
    float alpha_ab;
    float beta_ab;
    float e2_filt;
    float de2_filt;
    float e2_prev_smc;
    float z_int_smc;
    float active_time;
    bool  was_active;
    mission_state_t mission_state;
    float crossbar_timer;
    float state_timer;
    float line_loss_timer;
} hil_ctrl_ctx_t;

static void hil_ctx_reset(hil_ctrl_ctx_t *ctx)
{
    ctx->alpha_ab = 1.0f - expf(-2.0f * ZETA_FILT * WN_FILT * TS_SEC);
    ctx->beta_ab  = (WN_FILT * TS_SEC) * (WN_FILT * TS_SEC);
    observer_reset(&ctx->observer, 0.0f);
    motor_pi_reset(&ctx->motor_ctrl_L);
    motor_pi_reset(&ctx->motor_ctrl_R);
    ctx->e2_filt         = 0.0f;
    ctx->de2_filt        = 0.0f;
    ctx->e2_prev_smc     = 0.0f;
    ctx->z_int_smc       = 0.0f;
    ctx->active_time     = 0.0f;
    ctx->was_active      = false;
    ctx->mission_state   = MISSION_TO_LOAD;
    ctx->crossbar_timer  = 0.0f;
    ctx->state_timer     = 0.0f;
    ctx->line_loss_timer = 0.0f;
}

/* Thực hiện đúng 1 bước điều khiển Ts = 0.005 s đồng bộ 1:1 với gói UDP từ Simulink */
static void hil_control_step(
    hil_ctrl_ctx_t *ctx,
    const uint16_t adc_local[7],
    uint8_t mass_local,
    float vL_meas,
    float vR_meas,
    float *pwmL_out,
    float *pwmR_out)
{
    ctx->active_time += TS_SEC;

#if HIL_RUN_MODE == MODE_INNER_STEP_TEST
    (void)adc_local;
    ctx->was_active = true;
    float v_ref   = eval_inner_step_profile(ctx->active_time);
    float m_total = M_CHASSIS_KG + (float)mass_local;
    *pwmL_out = motor_pi_update(&ctx->motor_ctrl_L, v_ref, vL_meas, m_total);
    *pwmR_out = motor_pi_update(&ctx->motor_ctrl_R, v_ref, vR_meas, m_total);
#else
    /* Chế độ bám line (Bước 2 & 3) dùng cùng ngữ cảnh ctx */
    ctx->state_timer += TS_SEC;
    bool crossbar_raw = is_crossbar_detected(adc_local);
    ctx->crossbar_timer = crossbar_raw ? (ctx->crossbar_timer + TS_SEC) : 0.0f;
    bool crossbar_confirmed = (ctx->crossbar_timer >= CROSSBAR_DEBOUNCE_SEC);

    float e2_pred = ctx->e2_filt + ctx->de2_filt * TS_SEC;
    float e2_raw  = e2_pred;
    bool  line_ok = compute_lateral_error_gauss_newton(adc_local, mass_local, e2_pred, &e2_raw);

    if (!ctx->was_active && line_ok) {
        ctx->e2_filt     = e2_raw;
        ctx->e2_prev_smc = e2_raw;
        observer_reset(&ctx->observer, e2_raw);
        ctx->was_active  = true;
    }

    float v_act = 0.5f * (vR_meas + vL_meas);
    float w_act = (vR_meas - vL_meas) / TRACK_WIDTH_M;
    if (line_ok) {
        float res     = e2_raw - e2_pred;
        ctx->e2_filt  = e2_pred + ctx->alpha_ab * res;
        ctx->de2_filt = ctx->de2_filt + (ctx->beta_ab / TS_SEC) * res;
    } else {
        ctx->e2_filt   = e2_pred;
        ctx->de2_filt *= 0.90f;
    }
    observer_update(&ctx->observer, ctx->e2_filt, v_act, w_act);

    float v_ramp   = V_CRUISE_MAX * fminf(1.0f, ctx->active_time / 0.25f);
    float err_norm = fminf(fabsf(ctx->e2_filt) / 0.024f, 1.0f);
    float v_turn   = V_CRUISE_MAX - (V_CRUISE_MAX - V_CRUISE_MIN) * err_norm;
    float v_base   = fminf(v_ramp, v_turn);

    if (ctx->active_time > 0.25f && line_ok) {
        ctx->z_int_smc += KI_SMC_GAIN * ctx->e2_filt * TS_SEC;
        if (ctx->z_int_smc > Z_INT_MAX_M)  ctx->z_int_smc = Z_INT_MAX_M;
        if (ctx->z_int_smc < -Z_INT_MAX_M) ctx->z_int_smc = -Z_INT_MAX_M;
    }
    float de2_clean = (ctx->e2_filt - ctx->e2_prev_smc) / TS_SEC;
    ctx->e2_prev_smc = ctx->e2_filt;

    float sigma     = ctx->e2_filt + KD_RATE_SEC * de2_clean + ctx->z_int_smc;
    float sat_sigma = fminf(fmaxf(sigma / PHI_BL_M, -1.0f), 1.0f);
    float alpha_str = fminf(1.0f, fmaxf(0.15f, v_base / V_CRUISE_MAX));
    float w_raw     = (v_base * sinf(ctx->observer.e3_hat)
                      + alpha_str * (K1_KINEMATIC * ctx->e2_filt + K_SMC_GAIN * sat_sigma))
                      / SENSOR_LOOKAHEAD_M;
    w_raw = fminf(fmaxf(w_raw, -OMEGA_MAX_RADS), OMEGA_MAX_RADS);

    float half_B  = 0.5f * TRACK_WIDTH_M;
    float v_ref_L = fminf(fmaxf(v_base - half_B * w_raw, V_WHEEL_MIN), V_WHEEL_MAX);
    float v_ref_R = fminf(fmaxf(v_base + half_B * w_raw, V_WHEEL_MIN), V_WHEEL_MAX);
    float m_total = M_CHASSIS_KG + (float)mass_local;

    *pwmL_out = motor_pi_update(&ctx->motor_ctrl_L, v_ref_L, vL_meas, m_total);
    *pwmR_out = motor_pi_update(&ctx->motor_ctrl_R, v_ref_R, vR_meas, m_total);
    (void)crossbar_confirmed;
#endif
}

/* ============================================================================
 * 7. CORE 0 LWIP UDP NETWORK TASK
 * ============================================================================ */
static void send_udp_packet(int sock, const struct sockaddr_in *dest_addr, float pwmL, float pwmR)
{
    int16_t pwmL_scaled = (int16_t)lroundf(fminf(fmaxf(pwmL, -100.0f), 100.0f) * 100.0f);
    int16_t pwmR_scaled = (int16_t)lroundf(fminf(fmaxf(pwmR, -100.0f), 100.0f) * 100.0f);

    uint8_t tx_buf[HIL_TX_PACKET_LEN];
    tx_buf[0] = 0xBB;
    tx_buf[1] = (uint8_t)(pwmL_scaled & 0xFF);
    tx_buf[2] = (uint8_t)((pwmL_scaled >> 8) & 0xFF);
    tx_buf[3] = (uint8_t)(pwmR_scaled & 0xFF);
    tx_buf[4] = (uint8_t)((pwmR_scaled >> 8) & 0xFF);
    tx_buf[5] = tx_buf[1] ^ tx_buf[2] ^ tx_buf[3] ^ tx_buf[4];

    sendto(sock, tx_buf, HIL_TX_PACKET_LEN, 0,
           (const struct sockaddr *)dest_addr, sizeof(*dest_addr));
}

static void network_task(void *pvParameters)
{
    (void)pvParameters;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket create failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in local_addr = { 0 };
    local_addr.sin_family      = AF_INET;
    local_addr.sin_port        = htons(ESP_UDP_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    hil_ctrl_ctx_t ctx;
    hil_ctx_reset(&ctx);

    uint8_t rx_buf[64];
    struct sockaddr_in src_addr;
    struct sockaddr_in pc_dest = { 0 };
    pc_dest.sin_family = AF_INET;
    pc_dest.sin_port   = htons(PC_UDP_PORT);

    uint32_t last_rx_ms = 0;
    bool     active     = false;

    ESP_LOGI(TAG, "LOCKSTEP HIL UDP ready on port %d -> PC port %d", ESP_UDP_PORT, PC_UDP_PORT);

    for (;;) {
        socklen_t socklen = sizeof(src_addr);
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&src_addr, &socklen);

        uint32_t now_ms = get_time_ms();

        if (len == HIL_RX_PACKET_LEN && rx_buf[0] == 0xAA) {
            uint8_t cs = 0;
            for (int k = 1; k <= 19; k++) cs ^= rx_buf[k];

            if (cs == rx_buf[20]) {
                /* Tự động reset sạch khi bắt đầu lượt chạy Simulink mới */
                if (!active || (now_ms - last_rx_ms) > HIL_FAILSAFE_MS) {
                    hil_ctx_reset(&ctx);
                    active = true;
                    ESP_LOGI(TAG, "LOCKSTEP HIL SESSION START (payload = %u kg)", rx_buf[15]);
                }
                last_rx_ms = now_ms;

                uint16_t adc_tmp[7];
                for (int i = 0; i < 7; i++) {
                    adc_tmp[i] = (uint16_t)rx_buf[2 * i + 1]
                               | ((uint16_t)rx_buf[2 * i + 2] << 8);
                }
                uint8_t mass_tmp = rx_buf[15];
                int16_t vL_mms   = (int16_t)((uint16_t)rx_buf[16] | ((uint16_t)rx_buf[17] << 8));
                int16_t vR_mms   = (int16_t)((uint16_t)rx_buf[18] | ((uint16_t)rx_buf[19] << 8));

                float pwmL_tx = 0.0f, pwmR_tx = 0.0f;
                hil_control_step(&ctx, adc_tmp, mass_tmp,
                                 (float)vL_mms * 0.001f, (float)vR_mms * 0.001f,
                                 &pwmL_tx, &pwmR_tx);

                pc_dest.sin_addr = src_addr.sin_addr;
                send_udp_packet(sock, &pc_dest, pwmL_tx, pwmR_tx);
            }
        } else if (active && (now_ms - last_rx_ms) > HIL_FAILSAFE_MS) {
            active = false;
            hil_ctx_reset(&ctx);
            ESP_LOGW(TAG, "LOCKSTEP HIL SESSION ENDED -> Reset ready for next run");
        }
    }
}

/* ============================================================================
 * 8. APPLICATION ENTRY POINT
 * ============================================================================ */
void app_start(void)
{
    ESP_LOGI(TAG, "Reset reason: %d", (int)esp_reset_reason());

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
    }

    ESP_ERROR_CHECK(wifi_ap_init_base());
    ota_boot_window();

    /* Chạy network_task + hil_control_step đồng bộ 1:1 ở ưu tiên cao */
    xTaskCreatePinnedToCore(
        network_task, "HIL_Lockstep", 8192, NULL,
        configMAX_PRIORITIES - 2, NULL, 1);

    ESP_LOGI(TAG, "Lockstep HIL started");
}