/* app.c - 2-DOF PI Velocity Controller & 10s Verification over Wi-Fi */

#include "app.h"
#include "ota.h"
#include "wifi_ap.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "vel_ctrl.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "ctrl";

/* ---------------- Bản ghi nhị phân 14 bytes/mẫu (cả Motor A & B) ---------------- */
typedef struct __attribute__((packed)) {
    uint16_t seq;         /* 0..1999 (cl10) hoặc 0..5999 (id30) -> t = seq * 0.005 s */
    int16_t  vref_mms;    /* Vận tốc đặt [mm/s] (hoặc PWM*100 trong chế độ vòng hở) */
    int16_t  va_mms;      /* Vận tốc đo Motor A [mm/s] */
    int16_t  vb_mms;      /* Vận tốc đo Motor B [mm/s] */
    int16_t  pwm_a_x100;  /* PWM thực xuất Motor A [0.01 %] */
    int16_t  pwm_b_x100;  /* PWM thực xuất Motor B [0.01 %] */
    uint16_t dt_us_err;   /* |dt_actual - 5000| [us] */
} log_sample_t;

typedef enum {
    RUN_IDLE = 0,
    RUN_ACTIVE,
    RUN_DONE
} run_state_t;

typedef enum {
    MODE_CL10 = 0,        /* 10 s (2000 mẫu): Nghiệm thu vòng kín 2-DOF PI */
    MODE_ID30,            /* 30 s (6000 mẫu): Vòng hở nhận dạng */
    MODE_CHECK5           /*  5 s (1000 mẫu): Kiểm tra chiều & deadzone */
} run_mode_t;

typedef enum {
    TARGET_A = 0,
    TARGET_B,
    TARGET_AB
} motor_target_t;

static log_sample_t *s_log_buf = NULL;

static volatile run_state_t    s_state          = RUN_IDLE;
static volatile bool           s_start_req      = false;
static volatile bool           s_abort_req      = false;
static volatile run_mode_t     s_req_mode       = MODE_CL10;
static volatile motor_target_t s_req_target     = TARGET_AB;
static volatile uint32_t       s_total_samples  = 0;
static volatile uint32_t       s_logged_samples = 0;

static TaskHandle_t s_rt_task_handle = NULL;

/* Tham số mặc định 2-DOF PI cho gá đỡ (w_gc = 35 rad/s) */
static vel_ctrl_params_t s_params_a = {
    .kp = 226.94f, .ki = 2978.15f, .kff = 85.09f,
    .u_dz = 2.40f, .e_band = 0.035f, .u_max = 100.0f
};
static vel_ctrl_params_t s_params_b = {
    .kp = 271.89f, .ki = 2855.99f, .kff = 81.60f,
    .u_dz = 1.68f, .e_band = 0.035f, .u_max = 100.0f
};

static vel_ctrl_t s_ctrl_a;
static vel_ctrl_t s_ctrl_b;

static inline int16_t clamp_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lroundf(v);
}

/* ---------------- Quỹ đạo đặt 10s vòng kín [m/s] ---------------- */
static float get_cl10_vref_mps(uint32_t seq)
{
    float t = (float)seq * SYSID_TS_S;

    if (t < 0.30f) return 0.0f;
    if (t < 1.10f) {
        /* S-curve bậc 3: 0 -> 0.5 m/s trong 0.80 s (a_max = 0.9375 m/s^2) */
        float tau = (t - 0.30f) / 0.80f;
        return 0.5f * (3.0f * tau * tau - 2.0f * tau * tau * tau);
    }
    if (t < 2.00f) return 0.5f;   /* Ổn định tại v_nom */
    if (t < 3.30f) return 0.7f;   /* Bước 1: 0.5 -> 0.7 */
    if (t < 4.60f) return 0.5f;   /* Bước 2: 0.7 -> 0.5 */
    if (t < 5.90f) return 0.3f;   /* Bước 3: 0.5 -> 0.3 */
    if (t < 7.20f) return 0.5f;   /* Bước 4: 0.3 -> 0.5 */
    if (t < 8.50f) return 0.7f;   /* Bước 5: 0.5 -> 0.7 (lần 2) */
    return 0.0f;                  /* Phanh Short Brake dừng hẳn */
}

/* ---------------- Quỹ đạo vòng hở (dự phòng) [%PWM] ---------------- */
static float get_openloop_pwm(run_mode_t mode, uint32_t seq)
{
    uint32_t t_ms = seq * 5U;
    if (mode == MODE_CHECK5) {
        if (t_ms <  500) return  0.0f;
        if (t_ms < 1500) return 20.0f;
        if (t_ms < 3000) return 40.0f;
        if (t_ms < 4200) return 60.0f;
        return 0.0f;
    }
    if (t_ms <  1000) return  0.0f;
    if (t_ms <  3500) return 30.0f;
    if (t_ms <  6000) return 60.0f;
    if (t_ms <  8500) return 40.0f;
    if (t_ms < 11000) return 80.0f;
    if (t_ms < 13500) return 25.0f;
    if (t_ms < 16000) return 70.0f;
    if (t_ms < 18000) return  0.0f;
    if (t_ms < 20500) return 50.0f;
    if (t_ms < 23000) return 85.0f;
    if (t_ms < 25000) return 35.0f;
    if (t_ms < 27500) return 65.0f;
    if (t_ms < 29000) return 20.0f;
    return 0.0f;
}

/* ---------------- GPTimer 200 Hz ISR trên Core 1 ---------------- */
static bool IRAM_ATTR on_rt_timer_alarm(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata,
                                        void *user_ctx)
{
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)user_ctx, &hp_woken);
    return (hp_woken == pdTRUE);
}

/* ---------------- Core 1: Hard Real-Time Control Task ---------------- */
static void rt_control_task(void *pvParameters)
{
    encoder_init_on_core1();

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t t_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&t_cfg, &gptimer));

    gptimer_alarm_config_t a_cfg = {
        .reload_count = 0,
        .alarm_count  = SYSID_TS_US,
        .flags.auto_reload_on_alarm = true
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &a_cfg));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_rt_timer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    while (1) {
        if (!s_start_req) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        s_start_req = false;
        s_abort_req = false;

        run_mode_t     mode   = s_req_mode;
        motor_target_t target = s_req_target;
        uint32_t       n_samp = (mode == MODE_CL10)   ? 2000U :
                                (mode == MODE_CHECK5) ? 1000U : LOG_MAX_SAMPLES;

        /* Nạp mới tham số 2-DOF PI trước mỗi lượt chạy */
        vel_ctrl_init(&s_ctrl_a, &s_params_a);
        vel_ctrl_init(&s_ctrl_b, &s_params_b);

        s_total_samples  = n_samp;
        s_logged_samples = 0;
        s_state          = RUN_ACTIVE;

        encoder_reset();
        motor_stop_all_brake();
        gpio_set_level(PIN_USER_LED, 1);

        ulTaskNotifyTake(pdTRUE, 0);
        ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
        ESP_ERROR_CHECK(gptimer_start(gptimer));

        int64_t prev_tick_us = esp_timer_get_time();

        for (uint32_t seq = 0; seq < n_samp; seq++) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            int64_t now_us = esp_timer_get_time();
            int64_t dt_us  = now_us - prev_tick_us;
            prev_tick_us   = now_us;

            if (s_abort_req) {
                break;
            }

            /* 1. Đo vận tốc dài [m/s] theo M/T */
            float va_mps = encoder_sample_rpm(ENC_CH_A, now_us, NULL) * RPM_TO_MPS;
            float vb_mps = encoder_sample_rpm(ENC_CH_B, now_us, NULL) * RPM_TO_MPS;

            float ref_val = 0.0f;
            float pwm_a   = 0.0f;
            float pwm_b   = 0.0f;

            /* 2. Tính toán điều khiển và xuất PWM float trực tiếp lên 12-bit LEDC */
            if (mode == MODE_CL10) {
                float vref_mps = get_cl10_vref_mps(seq);
                ref_val = vref_mps * 1000.0f; /* Lưu mm/s */

                if (target == TARGET_A || target == TARGET_AB) {
                    pwm_a = vel_ctrl_step(&s_ctrl_a, vref_mps, va_mps, SYSID_TS_S);
                    motor_set_pwm(MOTOR_CH_A, pwm_a);
                } else {
                    motor_coast(MOTOR_CH_A);
                }

                if (target == TARGET_B || target == TARGET_AB) {
                    pwm_b = vel_ctrl_step(&s_ctrl_b, vref_mps, vb_mps, SYSID_TS_S);
                    motor_set_pwm(MOTOR_CH_B, pwm_b);
                } else {
                    motor_coast(MOTOR_CH_B);
                }
            } else {
                float pwm_cmd = get_openloop_pwm(mode, seq);
                ref_val = pwm_cmd * 100.0f;

                if (target == TARGET_A || target == TARGET_AB) {
                    pwm_a = pwm_cmd;
                    motor_set_pwm(MOTOR_CH_A, pwm_a);
                } else {
                    motor_coast(MOTOR_CH_A);
                }

                if (target == TARGET_B || target == TARGET_AB) {
                    pwm_b = pwm_cmd;
                    motor_set_pwm(MOTOR_CH_B, pwm_b);
                } else {
                    motor_coast(MOTOR_CH_B);
                }
            }

            /* 3. Ghi RAM (tuyệt đối không gọi Wi-Fi/ESP_LOG trong vòng 200 Hz) */
            int64_t err_us = (seq == 0) ? 0 : llabs(dt_us - SYSID_TS_US);
            if (err_us > 65535) err_us = 65535;

            log_sample_t *rec = &s_log_buf[seq];
            rec->seq        = (uint16_t)seq;
            rec->vref_mms   = clamp_i16(ref_val);
            rec->va_mms     = clamp_i16(va_mps * 1000.0f);
            rec->vb_mms     = clamp_i16(vb_mps * 1000.0f);
            rec->pwm_a_x100 = clamp_i16(pwm_a * 100.0f);
            rec->pwm_b_x100 = clamp_i16(pwm_b * 100.0f);
            rec->dt_us_err  = (uint16_t)err_us;

            s_logged_samples = seq + 1;
        }

        ESP_ERROR_CHECK(gptimer_stop(gptimer));

        motor_stop_all_brake();
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_stop_all_coast();
        gpio_set_level(PIN_USER_LED, 0);

        s_state = RUN_DONE;
    }
}

/* ---------------- Core 0: HTTP Endpoints cho MATLAB ---------------- */
static esp_err_t http_start_handler(httpd_req_t *req)
{
    if (s_state == RUN_ACTIVE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Test already running");
        return ESP_OK;
    }

    run_mode_t     mode   = MODE_CL10;
    motor_target_t target = TARGET_AB;

    char query[96] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[24];
        if (httpd_query_key_value(query, "mode", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "id30") == 0)        mode = MODE_ID30;
            else if (strcmp(val, "check5") == 0) mode = MODE_CHECK5;
            else                                 mode = MODE_CL10;
        }
        if (httpd_query_key_value(query, "motor", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "A") == 0 || strcmp(val, "a") == 0)      target = TARGET_A;
            else if (strcmp(val, "B") == 0 || strcmp(val, "b") == 0) target = TARGET_B;
            else                                                     target = TARGET_AB;
        }
    }

    s_req_mode   = mode;
    s_req_target = target;
    s_start_req  = true;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"started\"}");
    return ESP_OK;
}

static esp_err_t http_tune_handler(httpd_req_t *req)
{
    if (s_state == RUN_ACTIVE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cannot tune while running");
        return ESP_OK;
    }

    char query[192] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_OK;
    }

    char val[32];
    vel_ctrl_params_t *p = &s_params_a;
    if (httpd_query_key_value(query, "motor", val, sizeof(val)) == ESP_OK) {
        if (strcmp(val, "B") == 0 || strcmp(val, "b") == 0) {
            p = &s_params_b;
        }
    }

    if (httpd_query_key_value(query, "kp", val, sizeof(val)) == ESP_OK)    p->kp     = strtof(val, NULL);
    if (httpd_query_key_value(query, "ki", val, sizeof(val)) == ESP_OK)    p->ki     = strtof(val, NULL);
    if (httpd_query_key_value(query, "kff", val, sizeof(val)) == ESP_OK)   p->kff    = strtof(val, NULL);
    if (httpd_query_key_value(query, "udz", val, sizeof(val)) == ESP_OK)   p->u_dz   = strtof(val, NULL);
    if (httpd_query_key_value(query, "eband", val, sizeof(val)) == ESP_OK) p->e_band = strtof(val, NULL);

    char resp[192];
    snprintf(resp, sizeof(resp),
             "{\"ok\":true,\"kp\":%.2f,\"ki\":%.2f,\"kff\":%.2f,\"udz\":%.2f,\"eband\":%.4f}",
             p->kp, p->ki, p->kff, p->u_dz, p->e_band);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t http_stop_handler(httpd_req_t *req)
{
    s_abort_req = true;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"stopping\"}");
    return ESP_OK;
}

static esp_err_t http_status_handler(httpd_req_t *req)
{
    const char *st_str = (s_state == RUN_ACTIVE) ? "running" :
                         (s_state == RUN_DONE)   ? "done"    : "idle";
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"logged\":%lu,\"total\":%lu}",
             st_str,
             (unsigned long)s_logged_samples,
             (unsigned long)s_total_samples);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t http_dump_handler(httpd_req_t *req)
{
    if (s_state == RUN_ACTIVE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Still capturing");
        return ESP_OK;
    }

    uint32_t count = s_logged_samples;
    if (count == 0 || s_log_buf == NULL) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No data");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/octet-stream");
    const uint8_t *ptr = (const uint8_t *)s_log_buf;
    size_t remaining   = count * sizeof(log_sample_t);
    const size_t chunk = 4096;

    while (remaining > 0) {
        size_t n = (remaining > chunk) ? chunk : remaining;
        esp_err_t err = httpd_resp_send_chunk(req, (const char *)ptr, n);
        if (err != ESP_OK) return err;
        ptr       += n;
        remaining -= n;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = SYSID_HTTP_PORT;
    cfg.core_id     = 0;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/start",    .method = HTTP_GET, .handler = http_start_handler  },
        { .uri = "/tune",     .method = HTTP_GET, .handler = http_tune_handler   },
        { .uri = "/stop",     .method = HTTP_GET, .handler = http_stop_handler   },
        { .uri = "/status",   .method = HTTP_GET, .handler = http_status_handler },
        { .uri = "/dump.bin", .method = HTTP_GET, .handler = http_dump_handler   },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    ESP_LOGI(TAG, "HTTP server ready on http://192.168.4.1:%d", SYSID_HTTP_PORT);
}

static bool ensure_wifi_ap_online(void)
{
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        if (esp_wifi_init(&cfg) != ESP_OK) return false;
    }

    esp_wifi_stop();

    wifi_config_t ap_cfg = { 0 };
    strncpy((char *)ap_cfg.ap.ssid, SYSID_WIFI_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, SYSID_WIFI_PASS, sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len         = (uint8_t)strlen(SYSID_WIFI_SSID);
    ap_cfg.ap.channel          = 1;
    ap_cfg.ap.max_connection   = 4;
    ap_cfg.ap.authmode         = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.pmf_cfg.required = false;

    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK) return false;
    if (esp_wifi_set_config(WIFI_IF_AP, &ap_cfg) != ESP_OK) return false;
    if (esp_wifi_start() != ESP_OK) return false;
    esp_wifi_set_ps(WIFI_PS_NONE);
    return true;
}

static void blink_ready_led(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_set_level(PIN_USER_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
        gpio_set_level(PIN_USER_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

void app_start(void)
{
    /* 0. Khóa cầu H ngay khi boot */
    motor_init();
    motor_stop_all_coast();

    gpio_reset_pin(PIN_USER_LED);
    gpio_set_direction(PIN_USER_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_USER_LED, 0);

    /* Cấp phát động 1 lần (6000 * 14 = 84 KB) tránh vượt vùng .bss tĩnh */
    s_log_buf = (log_sample_t *)calloc(LOG_MAX_SAMPLES, sizeof(log_sample_t));
    if (!s_log_buf) {
        ESP_LOGE(TAG, "Khong du RAM cap phat log buffer!");
        return;
    }

    /* 1. Giữ nguyên cửa sổ OTA qua Wi-Fi */
    wifi_ap_init_base();
    ota_boot_window();

    /* 2. Bật SoftAP SysID và mở HTTP server trên Core 0 */
    if (ensure_wifi_ap_online()) {
        start_http_server();
        blink_ready_led();
    }

    /* 3. Chạy vòng điều khiển 200 Hz độc quyền trên Core 1 */
    xTaskCreatePinnedToCore(rt_control_task, "rt_ctrl", 4096, NULL, 20, &s_rt_task_handle, 1);
}