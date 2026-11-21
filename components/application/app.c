/* app.c - Motor Transfer Function Identification over Wi-Fi (Core 1 RT + Core 0 HTTP) */

#include "app.h"
#include "ota.h"
#include "wifi_ap.h"
#include "config.h"
#include "motor.h"
#include "encoder.h"

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

static const char *TAG = "sysid";

/* ---------------- Cấu trúc bản ghi nhị phân 12 bytes/mẫu ---------------- */
typedef struct __attribute__((packed)) {
    uint16_t seq;        /* 0..5999 -> t = seq * 0.005 s */
    int16_t  pwm_x100;   /* PWM thực xuất [0.01 %], -10000..+10000 */
    int16_t  rpm_a_x10;  /* Tốc độ Motor A [0.1 RPM], ±3276.7 RPM */
    int16_t  rpm_b_x10;  /* Tốc độ Motor B [0.1 RPM], ±3276.7 RPM */
    uint16_t dt_us_err;  /* |dt_actual - 5000| [us] để kiểm tra jitter */
    int16_t  enc_delta;  /* Số xung 4x trong 5 ms của kênh đang đo */
} sysid_sample_t;

typedef enum {
    SYSID_IDLE = 0,
    SYSID_RUNNING,
    SYSID_DONE
} sysid_state_t;

typedef enum {
    MODE_ID30 = 0,       /* 30 s (6000 mẫu): chuỗi bậc thang nhiều mức */
    MODE_CHECK5          /* 5 s (1000 mẫu): kiểm tra deadzone, chiều encoder, jitter */
} sysid_mode_t;

typedef enum {
    TARGET_MOTOR_A = 0,
    TARGET_MOTOR_B,
    TARGET_MOTOR_AB
} motor_target_t;

/* Bộ đệm RAM tĩnh: 6000 * 12 = 72,000 bytes (70.3 KB) */
static sysid_sample_t s_log_buf[SYSID_MAX_SAMPLES];

static volatile sysid_state_t  s_state          = SYSID_IDLE;
static volatile bool           s_start_req      = false;
static volatile bool           s_abort_req      = false;
static volatile sysid_mode_t   s_req_mode       = MODE_ID30;
static volatile motor_target_t s_req_target     = TARGET_MOTOR_A;
static volatile uint32_t       s_total_samples  = 0;
static volatile uint32_t       s_logged_samples = 0;

static TaskHandle_t s_rt_task_handle = NULL;

/* ---------------- Quỹ đạo kích thích PWM ---------------- */
/* Trả về PWM [%] tại chỉ số mẫu seq (Ts = 5 ms) */
static float get_excitation_pwm(sysid_mode_t mode, uint32_t seq)
{
    uint32_t t_ms = seq * 5U;

    if (mode == MODE_CHECK5) {
        /* 5 s: Kiểm tra ngưỡng ma sát tĩnh u_dz và chiều encoder */
        if (t_ms <  500) return  0.0f;
        if (t_ms < 1200) return 12.0f;
        if (t_ms < 2000) return 20.0f;
        if (t_ms < 3000) return 35.0f;
        if (t_ms < 4200) return 55.0f;
        return 0.0f;
    }

    /* MODE_ID30 (30 s): 0..20s Estimation (2/3), 20..30s Validation (1/3) */
    if (t_ms <  1000) return  0.0f;   /* Nền 0% */
    if (t_ms <  3500) return 30.0f;
    if (t_ms <  6000) return 60.0f;
    if (t_ms <  8500) return 40.0f;
    if (t_ms < 11000) return 80.0f;
    if (t_ms < 13500) return 25.0f;
    if (t_ms < 16000) return 70.0f;
    if (t_ms < 18000) return  0.0f;   /* Phanh về 0% kiểm tra suy giảm */
    if (t_ms < 20500) return 50.0f;
    if (t_ms < 23000) return 85.0f;   /* --- Tập Validation bắt đầu ~20 s --- */
    if (t_ms < 25000) return 35.0f;
    if (t_ms < 27500) return 65.0f;
    if (t_ms < 29000) return 20.0f;
    return 0.0f;                      /* Phanh dừng 1 s cuối */
}

static inline int16_t clamp_i16(float v)
{
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lroundf(v);
}

/* ---------------- Hardware Timer 200 Hz ISR trên Core 1 ---------------- */
static bool IRAM_ATTR on_rt_timer_alarm(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata,
                                        void *user_ctx)
{
    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)user_ctx, &hp_woken);
    return (hp_woken == pdTRUE);
}

/* ---------------- Core 1: Hard Real-Time Sampling Task ---------------- */
static void sysid_rt_task(void *pvParameters)
{
    /* Cài ngắt encoder ngay trên Core 1 */
    encoder_init_on_core1();

    /* Khởi tạo GPTimer 1 MHz, alarm mỗi 5000 us (200 Hz) trên Core 1 */
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

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_rt_timer_alarm
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, xTaskGetCurrentTaskHandle()));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    while (1) {
        /* Chờ lệnh Start từ HTTP server (Core 0) */
        if (!s_start_req) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        s_start_req = false;
        s_abort_req = false;

        sysid_mode_t   mode   = s_req_mode;
        motor_target_t target = s_req_target;
        uint32_t       n_samp = (mode == MODE_CHECK5) ? 1000U : SYSID_MAX_SAMPLES;

        s_total_samples  = n_samp;
        s_logged_samples = 0;
        s_state          = SYSID_RUNNING;

        encoder_reset();
        motor_stop_all_brake();
        gpio_set_level(PIN_USER_LED, 1);

        /* Xóa notify tồn đọng và bắt đầu nhịp 5 ms */
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

            /* 1. Đọc vận tốc M/T 2 kênh */
            int16_t da = 0, db = 0;
            float rpm_a = encoder_sample_rpm(ENC_CH_A, now_us, &da);
            float rpm_b = encoder_sample_rpm(ENC_CH_B, now_us, &db);

            /* 2. Tính & xuất PWM bước kế tiếp */
            float pwm_pct = get_excitation_pwm(mode, seq);
            if (target == TARGET_MOTOR_A) {
                motor_set_pwm(MOTOR_CH_A, pwm_pct);
                motor_coast(MOTOR_CH_B);
            } else if (target == TARGET_MOTOR_B) {
                motor_coast(MOTOR_CH_A);
                motor_set_pwm(MOTOR_CH_B, pwm_pct);
            } else {
                motor_set_pwm(MOTOR_CH_A, pwm_pct);
                motor_set_pwm(MOTOR_CH_B, pwm_pct);
            }

            /* 3. Ghi bản ghi 12 bytes vào RAM (không log UART/Wi-Fi) */
            int64_t err_us = (seq == 0) ? 0 : llabs(dt_us - SYSID_TS_US);
            if (err_us > 65535) err_us = 65535;

            sysid_sample_t *rec = &s_log_buf[seq];
            rec->seq       = (uint16_t)seq;
            rec->pwm_x100  = clamp_i16(pwm_pct * 100.0f);
            rec->rpm_a_x10 = clamp_i16(rpm_a * 10.0f);
            rec->rpm_b_x10 = clamp_i16(rpm_b * 10.0f);
            rec->dt_us_err = (uint16_t)err_us;
            rec->enc_delta = (target == TARGET_MOTOR_B) ? db : da;

            s_logged_samples = seq + 1;
        }

        ESP_ERROR_CHECK(gptimer_stop(gptimer));

        /* Phanh Short Brake 0.5 s rồi nhả về Coast để tránh nóng cầu H */
        motor_stop_all_brake();
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_stop_all_coast();
        gpio_set_level(PIN_USER_LED, 0);

        s_state = SYSID_DONE;
    }
}

/* ---------------- Core 0: HTTP Endpoints cho MATLAB ---------------- */
static esp_err_t http_start_handler(httpd_req_t *req)
{
    if (s_state == SYSID_RUNNING) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "Test already running");
        return ESP_OK;
    }

    sysid_mode_t   mode   = MODE_ID30;
    motor_target_t target = TARGET_MOTOR_A;

    char query[96] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[24];
        if (httpd_query_key_value(query, "mode", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "check5") == 0) mode = MODE_CHECK5;
        }
        if (httpd_query_key_value(query, "motor", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "B") == 0 || strcmp(val, "b") == 0) {
                target = TARGET_MOTOR_B;
            } else if (strcmp(val, "AB") == 0 || strcmp(val, "ab") == 0) {
                target = TARGET_MOTOR_AB;
            }
        }
    }

    s_req_mode   = mode;
    s_req_target = target;
    s_start_req  = true;

    const char *resp = "{\"ok\":true,\"msg\":\"started\"}";
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
    const char *st_str = (s_state == SYSID_RUNNING) ? "running" :
                         (s_state == SYSID_DONE)    ? "done"    : "idle";
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
    if (s_state == SYSID_RUNNING) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "Still capturing");
        return ESP_OK;
    }

    uint32_t count = s_logged_samples;
    if (count == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No data");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/octet-stream");
    const uint8_t *ptr = (const uint8_t *)s_log_buf;
    size_t remaining   = count * sizeof(sysid_sample_t);
    const size_t chunk = 4096;

    while (remaining > 0) {
        size_t n = (remaining > chunk) ? chunk : remaining;
        esp_err_t err = httpd_resp_send_chunk(req, (const char *)ptr, n);
        if (err != ESP_OK) {
            return err;
        }
        ptr       += n;
        remaining -= n;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static void start_sysid_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = SYSID_HTTP_PORT;
    cfg.core_id     = 0;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SysID HTTP server on port %d", SYSID_HTTP_PORT);
        return;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/start",    .method = HTTP_GET, .handler = http_start_handler  },
        { .uri = "/stop",     .method = HTTP_GET, .handler = http_stop_handler   },
        { .uri = "/status",   .method = HTTP_GET, .handler = http_status_handler },
        { .uri = "/dump.bin", .method = HTTP_GET, .handler = http_dump_handler   },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    ESP_LOGI(TAG, "SysID HTTP server ready: http://192.168.4.1:%d", SYSID_HTTP_PORT);
}

/* Đảm bảo SoftAP luôn bật sau khi ota_boot_window() kết thúc */
static bool ensure_wifi_ap_online(void)
{
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);

    if (err == ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "Wi-Fi da bi de-init sau OTA, dang khoi tao lai...");
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
            return false;
        }
    }

    /* Dung sach truoc khi cau hinh lai AP moi */
    esp_wifi_stop();

    wifi_config_t ap_cfg = { 0 };
    strncpy((char *)ap_cfg.ap.ssid, SYSID_WIFI_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, SYSID_WIFI_PASS, sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len         = (uint8_t)strlen(SYSID_WIFI_SSID);
    ap_cfg.ap.channel          = 1;
    ap_cfg.ap.max_connection   = 4;
    ap_cfg.ap.authmode         = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.pmf_cfg.required = false;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s (free heap: %lu)",
                 esp_err_to_name(err), (unsigned long)esp_get_free_heap_size());
        return false;
    }

    ESP_LOGI(TAG, "SysID AP ON: SSID=%s, free heap=%lu bytes",
             SYSID_WIFI_SSID, (unsigned long)esp_get_free_heap_size());
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
    ESP_LOGW("BOOT", "Reset reason: %d | Free heap: %lu",
             esp_reset_reason(), (unsigned long)esp_get_free_heap_size());

    /* 0. Khoa cau H ngay khi boot */
    motor_init();
    motor_stop_all_coast();

    gpio_reset_pin(PIN_USER_LED);
    gpio_set_direction(PIN_USER_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_USER_LED, 0);

    /* 1. Cua so OTA (15 giay) */
    wifi_ap_init_base();
    ota_boot_window();

    /* 2. Bat SoftAP rieng cho SysID va mo HTTP server cong 8080 */
    if (ensure_wifi_ap_online()) {
        start_sysid_http_server();
        blink_ready_led();
    } else {
        ESP_LOGE(TAG, "Khong the bat Wi-Fi SysID!");
    }

    /* 3. Task lay mau 200 Hz tren Core 1 */
    xTaskCreatePinnedToCore(sysid_rt_task, "sysid_rt", 4096, NULL, 20, &s_rt_task_handle, 1);
}