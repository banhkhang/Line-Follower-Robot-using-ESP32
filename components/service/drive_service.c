/*
 * drive_service.c - Trien khai Vong dieu khien van toc ben trong (Core 1, 200 Hz)
 *
 * Chi tiet kien truc va giao dien API xem trong drive_service.h va robot_params.h.
 */

#include "drive_service.h"
#include "config.h"
#include "robot_params.h"
#include "motor.h"
#include "encoder.h"
#include "vel_ctrl.h"

#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "drive_svc";

#define DRIVE_TASK_STACK_BYTES  4096
#define DRIVE_TASK_PRIORITY     20
#define DRIVE_TASK_CORE_ID      1

/* ---------------- Lenh chia se tu Core 0 -> Core 1 ---------------- */
typedef struct {
    drive_mode_t mode;
    float        v_cmd_mps;         /* Lenh van toc dai khung xe [m/s] */
    float        w_cmd_rads;        /* Lenh van toc goc khung xe [rad/s] */
    float        v_a_cmd_mps;       /* Lenh truc tiep banh A [m/s] */
    float        v_b_cmd_mps;       /* Lenh truc tiep banh B [m/s] */

    /* S-curve khoi hanh muot */
    bool         scurve_active;
    float        scurve_v_start;
    float        scurve_v_target;
    float        scurve_elapsed_s;
    float        scurve_duration_s;

    /* Cap nhat tham so 2-DOF PI */
    bool         params_dirty;
    bool         use_bench_params;
    float        m_load_kg;

    /* Reset moc odometry */
    bool         reset_odom_req;

    /* Dong bo tuy chon cho task vong ngoai */
    TaskHandle_t outer_task;
    uint32_t     outer_divider;
} drive_cmd_t;

static portMUX_TYPE  s_lock        = portMUX_INITIALIZER_UNLOCKED;
static bool          s_motors_init = false;
static bool          s_started     = false;
static TaskHandle_t  s_rt_task     = NULL;

static drive_cmd_t   s_cmd = {
    .mode              = DRIVE_MODE_COAST,
    .v_cmd_mps         = 0.0f,
    .w_cmd_rads        = 0.0f,
    .v_a_cmd_mps       = 0.0f,
    .v_b_cmd_mps       = 0.0f,
    .scurve_active     = false,
    .scurve_v_start    = 0.0f,
    .scurve_v_target   = 0.0f,
    .scurve_elapsed_s  = 0.0f,
    .scurve_duration_s = ROBOT_SCURVE_DEFAULT_T_S,
    .params_dirty      = true,
    .use_bench_params  = false,
    .m_load_kg         = 0.0f,
    .reset_odom_req    = false,
    .outer_task        = NULL,
    .outer_divider     = 0
};

static drive_state_t s_state = { 0 };

/* ---------------- Ham phu tro ---------------- */

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float drive_service_scurve_eval(float tau, float v_start, float v_target)
{
    if (tau <= 0.0f) return v_start;
    if (tau >= 1.0f) return v_target;
    /* Da thuc Hermite bac 3: dao ham bang 0 tai tau=0 va tau=1, a_max = 1.5*dv/T */
    float s = tau * tau * (3.0f - 2.0f * tau);
    return v_start + (v_target - v_start) * s;
}

/* ---------------- GPTimer 200 Hz ISR tren Core 1 ---------------- */
static bool IRAM_ATTR on_inner_timer_alarm(gptimer_handle_t timer,
                                           const gptimer_alarm_event_data_t *edata,
                                           void *user_ctx)
{
    (void)timer;
    (void)edata;
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)user_ctx, &hp_woken);
    return (hp_woken == pdTRUE);
}

/* ---------------- Core 1: Hard Real-Time Inner Velocity Loop (200 Hz) ---------------- */
static void drive_rt_task(void *pvParameters)
{
    (void)pvParameters;

    /* 1. Cai dat ngat Encoder tren chinh Core 1 de tach biet voi Wi-Fi tren Core 0 */
    encoder_init_on_core1();

    /* 2. Khoi tao 2 bo 2-DOF PI doc lap cho Motor A (trai) va Motor B (phai) */
    vel_ctrl_t ctrl_a, ctrl_b;
    vel_ctrl_params_t pa, pb;
    vel_ctrl_get_mat_params(0.0f, &pa, &pb);
    vel_ctrl_init(&ctrl_a, &pa);
    vel_ctrl_init(&ctrl_b, &pb);

    /* 3. Khoi tao bo dinh thoi cung GPTimer 1 MHz, bao thuc moi 5000 us (200 Hz) */
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t t_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&t_cfg, &gptimer));

    gptimer_alarm_config_t a_cfg = {
        .reload_count               = 0,
        .alarm_count                = INNER_LOOP_TS_US,
        .flags.auto_reload_on_alarm = true
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &a_cfg));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_inner_timer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    int32_t      odom_zero_a  = encoder_get_count(ENC_CH_A);
    int32_t      odom_zero_b  = encoder_get_count(ENC_CH_B);
    drive_mode_t prev_mode    = DRIVE_MODE_COAST;
    int64_t      prev_tick_us = esp_timer_get_time();
    uint32_t     tick_cnt     = 0;

    for (;;) {
        /* Cho dung nhip 5.000 ms tu GPTimer ISR */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        int64_t dt_us  = now_us - prev_tick_us;
        prev_tick_us   = now_us;
        tick_cnt++;

        /* --- Buoc 1: Doc encoder M/T 1x & tinh dong hoc thuan + odometry --- */
        float rpm_a  = encoder_sample_rpm(ENC_CH_A, now_us, NULL);
        float rpm_b  = encoder_sample_rpm(ENC_CH_B, now_us, NULL);
        float va_mps = rpm_a * ROBOT_RPM_TO_MPS;
        float vb_mps = rpm_b * ROBOT_RPM_TO_MPS;

        int32_t cnt_a = encoder_get_count(ENC_CH_A);
        int32_t cnt_b = encoder_get_count(ENC_CH_B);

        /* --- Buoc 2: Nhan lenh moi nhat tu Core 0 (spinlock ngan < 1 us) --- */
        drive_cmd_t cmd;
        portENTER_CRITICAL(&s_lock);
        if (s_cmd.reset_odom_req) {
            odom_zero_a = cnt_a;
            odom_zero_b = cnt_b;
            s_cmd.reset_odom_req = false;
        }
        if (s_cmd.scurve_active) {
            s_cmd.scurve_elapsed_s += INNER_LOOP_TS_S;
            if (s_cmd.scurve_elapsed_s >= s_cmd.scurve_duration_s) {
                s_cmd.scurve_elapsed_s = s_cmd.scurve_duration_s;
                s_cmd.scurve_active    = false;
            }
        }
        cmd = s_cmd;
        s_cmd.params_dirty = false;
        portEXIT_CRITICAL(&s_lock);

        /* Dong hoc thuan vi sai (Motor A = PHAI phia S7, Motor B = TRAI phia S1) */
        float v_act = 0.5f * (va_mps + vb_mps);
        float w_act = (va_mps - vb_mps) / ROBOT_TRACK_WIDTH_M;
        float s_a   = (float)(cnt_a - odom_zero_a) * ENC_DIST_PER_PULSE_1X_M;
        float s_b   = (float)(cnt_b - odom_zero_b) * ENC_DIST_PER_PULSE_1X_M;
        float s_m   = 0.5f * (s_a + s_b);

        /* --- Buoc 3: Cap nhat tham so 2-DOF PI neu co thay doi tai trong --- */
        if (cmd.params_dirty) {
            if (cmd.use_bench_params) {
                vel_ctrl_get_bench_params(&ctrl_a.p, &ctrl_b.p);
            } else {
                vel_ctrl_apply_payload(&ctrl_a, &ctrl_b, cmd.m_load_kg);
            }
        }

        /* Xoa tich phan khi vua chuyen tu trang thai dung (Coast/Brake) sang chay */
        if (cmd.mode != prev_mode) {
            if (cmd.mode == DRIVE_MODE_COAST || cmd.mode == DRIVE_MODE_BRAKE ||
                prev_mode == DRIVE_MODE_COAST || prev_mode == DRIVE_MODE_BRAKE) {
                vel_ctrl_reset(&ctrl_a);
                vel_ctrl_reset(&ctrl_b);
            }
            prev_mode = cmd.mode;
        }

        /* --- Buoc 4: Tinh van toc dat hai banh & xuat PWM 12-bit Slow-Decay --- */
        float v_eff  = 0.0f;
        float w_eff  = 0.0f;
        float va_ref = 0.0f;
        float vb_ref = 0.0f;
        float pwm_a  = 0.0f;
        float pwm_b  = 0.0f;

        switch (cmd.mode) {
            case DRIVE_MODE_COAST:
                motor_stop_all_coast();
                break;

            case DRIVE_MODE_BRAKE:
                motor_stop_all_brake();
                break;

            case DRIVE_MODE_CHASSIS_VEL: {
                v_eff = cmd.v_cmd_mps;
                w_eff = clampf(cmd.w_cmd_rads, -ROBOT_W_MAX_RADS, ROBOT_W_MAX_RADS);

                /*
                 * Neu S-curve dang hoat dong khi khoi hanh tien: lay min voi v_cmd
                 * de bao dam khoi dong muot (a_max <= 1.0 m/s^2) nhung khong bao gio
                 * can tro lenh giam toc/phanh tu vong ngoai.
                 */
                if (cmd.scurve_active && cmd.scurve_duration_s > 1e-4f) {
                    float tau   = cmd.scurve_elapsed_s / cmd.scurve_duration_s;
                    float v_sc  = drive_service_scurve_eval(tau, cmd.scurve_v_start, cmd.scurve_v_target);
                    if (v_eff > 0.0f && v_sc < v_eff) {
                        v_eff = v_sc;
                    }
                }

                /*
                 * Dong hoc nghich vi sai (Motor A = PHAI phia S7, Motor B = TRAI phia S1):
                 *   w > 0 (re TRAI / CCW) -> banh PHAI (A) nhanh hon, banh TRAI (B) cham hon:
                 *     v_A (PHAI) = v + (B/2)*w,   v_B (TRAI) = v - (B/2)*w
                 * Khi chay tien (v_eff > 0): kep banh trong >= 0 m/s de chong van quay tai cho.
                 */
                float v_lo = (v_eff > 0.0f) ? 0.0f : -ROBOT_V_WHEEL_ABS_MAX_MPS;
                va_ref = clampf(v_eff + ROBOT_HALF_TRACK_M * w_eff,
                                v_lo, ROBOT_V_WHEEL_ABS_MAX_MPS);
                vb_ref = clampf(v_eff - ROBOT_HALF_TRACK_M * w_eff,
                                v_lo, ROBOT_V_WHEEL_ABS_MAX_MPS);

                pwm_a = vel_ctrl_step(&ctrl_a, va_ref, va_mps, INNER_LOOP_TS_S);
                pwm_b = vel_ctrl_step(&ctrl_b, vb_ref, vb_mps, INNER_LOOP_TS_S);
                motor_set_pwm(MOTOR_CH_A, pwm_a);
                motor_set_pwm(MOTOR_CH_B, pwm_b);
                break;
            }

            case DRIVE_MODE_WHEEL_VEL:
                va_ref = clampf(cmd.v_a_cmd_mps,
                                -ROBOT_V_WHEEL_ABS_MAX_MPS, ROBOT_V_WHEEL_ABS_MAX_MPS);
                vb_ref = clampf(cmd.v_b_cmd_mps,
                                -ROBOT_V_WHEEL_ABS_MAX_MPS, ROBOT_V_WHEEL_ABS_MAX_MPS);
                v_eff  = 0.5f * (va_ref + vb_ref);
                w_eff  = (va_ref - vb_ref) / ROBOT_TRACK_WIDTH_M;

                pwm_a = vel_ctrl_step(&ctrl_a, va_ref, va_mps, INNER_LOOP_TS_S);
                pwm_b = vel_ctrl_step(&ctrl_b, vb_ref, vb_mps, INNER_LOOP_TS_S);
                motor_set_pwm(MOTOR_CH_A, pwm_a);
                motor_set_pwm(MOTOR_CH_B, pwm_b);
                break;
        }

        /* --- Buoc 5: Xuat anh chup trang thai an toan cho Core 0 --- */
        int64_t err_us = (tick_cnt <= 1U) ? 0 : llabs(dt_us - (int64_t)INNER_LOOP_TS_US);
        if (err_us > 65535) err_us = 65535;

        portENTER_CRITICAL(&s_lock);
        s_state.mode          = cmd.mode;
        s_state.v_a_mps       = va_mps;
        s_state.v_b_mps       = vb_mps;
        s_state.rpm_a         = rpm_a;
        s_state.rpm_b         = rpm_b;
        s_state.v_mps         = v_act;
        s_state.w_rads        = w_act;
        s_state.s_m           = s_m;
        s_state.v_cmd_eff_mps = v_eff;
        s_state.w_cmd_rads    = w_eff;
        s_state.v_a_ref_mps   = va_ref;
        s_state.v_b_ref_mps   = vb_ref;
        s_state.pwm_a_pct     = pwm_a;
        s_state.pwm_b_pct     = pwm_b;
        s_state.m_load_kg     = cmd.m_load_kg;
        s_state.scurve_active = cmd.scurve_active;
        s_state.tick_count    = tick_cnt;
        s_state.dt_us_err     = (uint16_t)err_us;
        portEXIT_CRITICAL(&s_lock);

        /* --- Buoc 6: Bao thuc task vong ngoai dong bo pha (neu co dang ky) --- */
        if (cmd.outer_task != NULL && cmd.outer_divider > 0U &&
            (tick_cnt % cmd.outer_divider) == 0U) {
            xTaskNotifyGive(cmd.outer_task);
        }
    }
}

/* ---------------- Public API (Thread-Safe) ---------------- */

void drive_service_lock_motors(void)
{
    if (!s_motors_init) {
        motor_init();
        s_motors_init = true;
    }
    motor_stop_all_coast();
}

esp_err_t drive_service_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    /* Dam bao cau H da duoc khoi tao & khoa o trang thai Coast truoc khi chay task */
    drive_service_lock_motors();

    BaseType_t ok = xTaskCreatePinnedToCore(
        drive_rt_task,
        "drive_rt_200hz",
        DRIVE_TASK_STACK_BYTES,
        NULL,
        DRIVE_TASK_PRIORITY,
        &s_rt_task,
        DRIVE_TASK_CORE_ID
    );
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Khong the tao task vong trong 200 Hz tren Core 1");
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    ESP_LOGI(TAG, "Inner velocity loop ready (Core 1, %u Hz, B=%.3f m, D=%.3f m)",
             (unsigned)INNER_LOOP_FS_HZ, ROBOT_TRACK_WIDTH_M, ROBOT_WHEEL_DIAMETER_M);
    return ESP_OK;
}

void drive_service_stop_coast(void)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.mode          = DRIVE_MODE_COAST;
    s_cmd.v_cmd_mps     = 0.0f;
    s_cmd.w_cmd_rads    = 0.0f;
    s_cmd.v_a_cmd_mps   = 0.0f;
    s_cmd.v_b_cmd_mps   = 0.0f;
    s_cmd.scurve_active = false;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_stop_brake(void)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.mode          = DRIVE_MODE_BRAKE;
    s_cmd.v_cmd_mps     = 0.0f;
    s_cmd.w_cmd_rads    = 0.0f;
    s_cmd.v_a_cmd_mps   = 0.0f;
    s_cmd.v_b_cmd_mps   = 0.0f;
    s_cmd.scurve_active = false;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_set_chassis_vel(float v_mps, float w_rads)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.mode       = DRIVE_MODE_CHASSIS_VEL;
    s_cmd.v_cmd_mps  = v_mps;
    s_cmd.w_cmd_rads = w_rads;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_start_scurve(float v_target_mps, float duration_s)
{
    if (duration_s < INNER_LOOP_TS_S) {
        duration_s = INNER_LOOP_TS_S;
    }

    portENTER_CRITICAL(&s_lock);
    float v_now = s_state.v_mps;
    if (v_now < 0.0f) v_now = 0.0f;
    s_cmd.mode              = DRIVE_MODE_CHASSIS_VEL;
    s_cmd.v_cmd_mps         = v_target_mps;
    s_cmd.scurve_v_start    = v_now;
    s_cmd.scurve_v_target   = v_target_mps;
    s_cmd.scurve_elapsed_s  = 0.0f;
    s_cmd.scurve_duration_s = duration_s;
    s_cmd.scurve_active     = true;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_cancel_scurve(void)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.scurve_active = false;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_set_wheel_vel(float v_a_mps, float v_b_mps)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.mode          = DRIVE_MODE_WHEEL_VEL;
    s_cmd.v_a_cmd_mps   = v_a_mps;
    s_cmd.v_b_cmd_mps   = v_b_mps;
    s_cmd.scurve_active = false;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_set_payload_kg(float m_load_kg)
{
    if (m_load_kg < ROBOT_PAYLOAD_MIN_KG)       m_load_kg = ROBOT_PAYLOAD_MIN_KG;
    if (m_load_kg > ROBOT_PAYLOAD_CLAMP_MAX_KG) m_load_kg = ROBOT_PAYLOAD_CLAMP_MAX_KG;

    portENTER_CRITICAL(&s_lock);
    s_cmd.m_load_kg        = m_load_kg;
    s_cmd.use_bench_params = false;
    s_cmd.params_dirty     = true;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_use_bench_params(void)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.use_bench_params = true;
    s_cmd.params_dirty     = true;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_reset_odometry(void)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.reset_odom_req = true;
    portEXIT_CRITICAL(&s_lock);
}

void drive_service_get_state(drive_state_t *out)
{
    if (out == NULL) return;
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}

bool drive_service_is_stopped(void)
{
    float ra, rb;
    portENTER_CRITICAL(&s_lock);
    ra = s_state.rpm_a;
    rb = s_state.rpm_b;
    portEXIT_CRITICAL(&s_lock);
    return (fabsf(ra) < ROBOT_STOP_RPM_THR) && (fabsf(rb) < ROBOT_STOP_RPM_THR);
}

void drive_service_register_outer_notify(TaskHandle_t task_handle, uint32_t divider)
{
    portENTER_CRITICAL(&s_lock);
    s_cmd.outer_task    = task_handle;
    s_cmd.outer_divider = divider;
    portEXIT_CRITICAL(&s_lock);
}
