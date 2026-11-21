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
static const float KP_BASE          = 460.0f;
static const float KI_BASE          = 2600.0f;
static const float KFF_GAIN         = 86.23f;
static const float PWM_DEADZONE_0   = 2.10f;
static const float INT_BAND_BASE    = 0.035f;

/* Variable-mass coupling parameters */
static const float TAU_0_SEC        = 0.055f;
static const float GAMMA_INERTIA    = 0.0384f;
static const float K_PWM_ROLL       = 1.55f;
static const float ADAPT_EXP_BETA   = 0.65f;

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

/* Damped Integral SMC / Kinematic Steering Law */
static const float K1_KINEMATIC     = 4.20f;
static const float K_SMC_GAIN       = 0.18f;
static const float PHI_BL_M         = 0.012f;
static const float KI_SMC_GAIN      = 1.40f;
static const float KD_RATE_SEC      = 0.018f;
static const float Z_INT_MAX_M      = 0.015f;
static const float OMEGA_MAX_RADS   = 2.80f;

/* Speed profile limits */
static const float V_CRUISE_MAX     = 0.50f;
static const float V_CRUISE_MIN     = 0.30f;
static const float V_WHEEL_MAX      = 0.72f;
static const float V_WHEEL_MIN      = 0.08f;

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

    float accel_max = 2.5f / sqrtf(alpha_m);
    float max_dv    = accel_max * TS_SEC;
    float dv        = v_ref_target - pi->v_ref_shaped;
    if (dv > max_dv)       dv = max_dv;
    else if (dv < -max_dv) dv = -max_dv;
    pi->v_ref_shaped += dv;

    float v_ref = pi->v_ref_shaped;
    float err   = v_ref - v_meas;

    float scale_p  = powf(alpha_m, ADAPT_EXP_BETA);
    float kp_eff   = KP_BASE * scale_p;
    float ki_eff   = KI_BASE * (scale_p / alpha_m);
    float int_band = INT_BAND_BASE * sqrtf(alpha_m);

    float pwm_comp = PWM_DEADZONE_0 + K_PWM_ROLL * m_total;
    float u_ff     = 0.0f;
    if (v_ref > 0.005f)       u_ff = KFF_GAIN * v_ref + pwm_comp;
    else if (v_ref < -0.005f) u_ff = KFF_GAIN * v_ref - pwm_comp;

    float u_p         = kp_eff * err;
    float pwm_min_dyn = (v_ref > 0.05f) ? 0.0f : PWM_MIN_REVERSE;

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

static void control_task(void *pvParameters)
{
    (void)pvParameters;

    const float alpha_ab = 1.0f - expf(-2.0f * ZETA_FILT * WN_FILT * TS_SEC);
    const float beta_ab  = (WN_FILT * TS_SEC) * (WN_FILT * TS_SEC);

    adaptive_observer_t observer;
    mass_adaptive_pi_t  motor_ctrl_L;
    mass_adaptive_pi_t  motor_ctrl_R;

    observer_reset(&observer, 0.0f);
    motor_pi_reset(&motor_ctrl_L);
    motor_pi_reset(&motor_ctrl_R);

    float e2_filt     = 0.0f;
    float de2_filt    = 0.0f;
    float e2_prev_smc = 0.0f;
    float z_int_smc   = 0.0f;
    float active_time = 0.0f;
    bool  was_active  = false;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t adc_local[7];
        uint8_t  mass_local;
        float    vL_meas, vR_meas;
        uint32_t last_rx;

        taskENTER_CRITICAL(&s_state_mux);
        memcpy(adc_local, s_hil_state.sensor_adc, sizeof(adc_local));
        mass_local = s_hil_state.mass_kg;
        vL_meas    = s_hil_state.vL_meas_ms;
        vR_meas    = s_hil_state.vR_meas_ms;
        last_rx    = s_hil_state.last_rx_ms;
        taskEXIT_CRITICAL(&s_state_mux);

        uint32_t now_ms = get_time_ms();
        bool link_ok = (last_rx > 0) && ((now_ms - last_rx) <= HIL_FAILSAFE_MS);

        if (!link_ok) {
            if (was_active) {
                motor_pi_reset(&motor_ctrl_L);
                motor_pi_reset(&motor_ctrl_R);
                observer_reset(&observer, 0.0f);
                e2_filt     = 0.0f;
                de2_filt    = 0.0f;
                e2_prev_smc = 0.0f;
                z_int_smc   = 0.0f;
                active_time = 0.0f;
                was_active  = false;
            }
            taskENTER_CRITICAL(&s_state_mux);
            s_hil_state.pwmL_cmd    = 0.0f;
            s_hil_state.pwmR_cmd    = 0.0f;
            s_hil_state.link_active = false;
            taskEXIT_CRITICAL(&s_state_mux);
            continue;
        }

        active_time += TS_SEC;

        float e2_pred = e2_filt + de2_filt * TS_SEC;
        float e2_raw  = e2_pred;
        bool  line_ok = compute_lateral_error_gauss_newton(adc_local, mass_local, e2_pred, &e2_raw);

        if (!was_active && line_ok) {
            e2_filt     = e2_raw;
            e2_prev_smc = e2_raw;
            observer_reset(&observer, e2_raw);
            was_active  = true;
        }

        if (line_ok) {
            float res = e2_raw - e2_pred;
            e2_filt   = e2_pred + alpha_ab * res;
            de2_filt  = de2_filt + (beta_ab / TS_SEC) * res;
        } else {
            e2_filt   = e2_pred;
            de2_filt *= 0.90f;
        }

        float v_act = 0.5f * (vR_meas + vL_meas);
        float w_act = (vR_meas - vL_meas) / TRACK_WIDTH_M;
        observer_update(&observer, e2_filt, v_act, w_act);
        float e3_hat = observer.e3_hat;

        float v_ramp   = V_CRUISE_MAX * fminf(1.0f, active_time / 0.25f);
        float err_norm = fminf(fabsf(e2_filt) / 0.024f, 1.0f);
        float v_turn   = V_CRUISE_MAX - (V_CRUISE_MAX - V_CRUISE_MIN) * err_norm;
        float v_base   = fminf(v_ramp, v_turn);

        if (active_time > 0.25f && line_ok) {
            z_int_smc += KI_SMC_GAIN * e2_filt * TS_SEC;
            if (z_int_smc > Z_INT_MAX_M)  z_int_smc = Z_INT_MAX_M;
            if (z_int_smc < -Z_INT_MAX_M) z_int_smc = -Z_INT_MAX_M;
        }

        float de2_clean = (e2_filt - e2_prev_smc) / TS_SEC;
        e2_prev_smc     = e2_filt;

        float sigma     = e2_filt + KD_RATE_SEC * de2_clean + z_int_smc;
        float sat_sigma = sigma / PHI_BL_M;
        if (sat_sigma > 1.0f)  sat_sigma = 1.0f;
        if (sat_sigma < -1.0f) sat_sigma = -1.0f;

        float alpha_steer = fminf(1.0f, fmaxf(0.15f, v_base / V_CRUISE_MAX));
        float w_raw = (v_base * sinf(e3_hat) + alpha_steer * (K1_KINEMATIC * e2_filt + K_SMC_GAIN * sat_sigma))
                    / SENSOR_LOOKAHEAD_M;

        if (w_raw > OMEGA_MAX_RADS)  w_raw = OMEGA_MAX_RADS;
        if (w_raw < -OMEGA_MAX_RADS) w_raw = -OMEGA_MAX_RADS;

        float half_B  = 0.5f * TRACK_WIDTH_M;
        float v_ref_L = fminf(fmaxf(v_base - half_B * w_raw, V_WHEEL_MIN), V_WHEEL_MAX);
        float v_ref_R = fminf(fmaxf(v_base + half_B * w_raw, V_WHEEL_MIN), V_WHEEL_MAX);

        float m_total = M_CHASSIS_KG + (float)mass_local;
        float pwm_L   = motor_pi_update(&motor_ctrl_L, v_ref_L, vL_meas, m_total);
        float pwm_R   = motor_pi_update(&motor_ctrl_R, v_ref_R, vR_meas, m_total);

        taskENTER_CRITICAL(&s_state_mux);
        s_hil_state.pwmL_cmd    = pwm_L;
        s_hil_state.pwmR_cmd    = pwm_R;
        s_hil_state.link_active = true;
        taskEXIT_CRITICAL(&s_state_mux);
    }
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
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in local_addr = { 0 };
    local_addr.sin_family      = AF_INET;
    local_addr.sin_port        = htons(ESP_UDP_PORT);
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed (port %d): errno %d", ESP_UDP_PORT, errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    /* Timeout 50 ms để task không treo vô hạn khi Simulink dừng */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    ESP_LOGI(TAG, "HIL UDP listening on port %d -> reply to PC port %d",
             ESP_UDP_PORT, PC_UDP_PORT);

    uint8_t rx_buf[64];
    struct sockaddr_in src_addr;
    struct sockaddr_in pc_dest = { 0 };
    pc_dest.sin_family = AF_INET;
    pc_dest.sin_port   = htons(PC_UDP_PORT);

    bool prev_active   = false;
    uint8_t prev_mass  = 0xFF;

    for (;;) {
        socklen_t socklen = sizeof(src_addr);
        int len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0,
                           (struct sockaddr *)&src_addr, &socklen);

        if (len == HIL_RX_PACKET_LEN && rx_buf[0] == 0xAA) {
            uint8_t cs = 0;
            for (int k = 1; k <= 19; k++) cs ^= rx_buf[k];

            if (cs == rx_buf[20]) {
                uint16_t adc_tmp[7];
                for (int i = 0; i < 7; i++) {
                    adc_tmp[i] = (uint16_t)rx_buf[2 * i + 1]
                               | ((uint16_t)rx_buf[2 * i + 2] << 8);
                }
                uint8_t mass_tmp = rx_buf[15];
                int16_t vL_mms   = (int16_t)((uint16_t)rx_buf[16] | ((uint16_t)rx_buf[17] << 8));
                int16_t vR_mms   = (int16_t)((uint16_t)rx_buf[18] | ((uint16_t)rx_buf[19] << 8));

                float pwmL_tx, pwmR_tx;
                taskENTER_CRITICAL(&s_state_mux);
                memcpy(s_hil_state.sensor_adc, adc_tmp, sizeof(adc_tmp));
                s_hil_state.mass_kg    = mass_tmp;
                s_hil_state.vL_meas_ms = (float)vL_mms * 0.001f;
                s_hil_state.vR_meas_ms = (float)vR_mms * 0.001f;
                s_hil_state.last_rx_ms = get_time_ms();
                pwmL_tx = s_hil_state.pwmL_cmd;
                pwmR_tx = s_hil_state.pwmR_cmd;
                taskEXIT_CRITICAL(&s_state_mux);

                /* Gửi trả về đúng IP của PC (Simulink) tại cổng 8888 */
                pc_dest.sin_addr = src_addr.sin_addr;
                send_udp_packet(sock, &pc_dest, pwmL_tx, pwmR_tx);
            }
        }

        /* Chỉ in log khi trạng thái kết nối hoặc khối lượng thay đổi */
        bool active;
        uint8_t mass_now;
        taskENTER_CRITICAL(&s_state_mux);
        active   = s_hil_state.link_active;
        mass_now = s_hil_state.mass_kg;
        taskEXIT_CRITICAL(&s_state_mux);

        if (active != prev_active || (active && mass_now != prev_mass)) {
            if (active) {
                ESP_LOGI(TAG, "HIL ACTIVE | payload = %u kg", mass_now);
            } else {
                ESP_LOGW(TAG, "HIL IDLE / waiting for Simulink...");
            }
            prev_active = active;
            prev_mass   = mass_now;
        }
    }
}

/* ============================================================================
 * 8. APPLICATION ENTRY POINT
 * ============================================================================ */
void app_start(void)
{
    ESP_LOGI(TAG, "Reset reason: %d", (int)esp_reset_reason());

    /* Xác nhận firmware hợp lệ nếu bật Rollback trong menuconfig */
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
    }

    /* 1. Khởi tạo tầng mạng nền */
    ESP_ERROR_CHECK(wifi_ap_init_base());

    /* 2. Mở cửa sổ OTA (SoftAP ESP32S3_HIL_BOT, giữ nguyên AP khi kết thúc) */
    ota_boot_window();

    /* 3. Khởi chạy 2 task HIL trên 2 nhân riêng biệt */
    xTaskCreatePinnedToCore(
        control_task, "HIL_Ctrl200Hz", 6144, NULL,
        configMAX_PRIORITIES - 2, &s_control_task_handle, 1);

    xTaskCreatePinnedToCore(
        network_task, "HIL_UDP", 6144, NULL,
        configMAX_PRIORITIES - 3, NULL, 0);

    /* 4. Bật bộ định thời cứng 5000 us (200 Hz) */
    const esp_timer_create_args_t timer_args = {
        .callback        = &on_control_timer,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "hil_200hz",
    };
    esp_timer_handle_t control_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &control_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(control_timer, 5000));

    ESP_LOGI(TAG, "HIL 200 Hz system started on ESP32-S3");
}