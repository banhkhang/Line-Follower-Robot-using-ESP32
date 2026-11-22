#include "app.h"
#include "ota.h"
#include "wifi_ap.h"
#include "config.h"
#include "loadcell_service.h"

#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "app_cal";

/* Keep DRV8871 outputs safely OFF (Coast) via inverting 6N137 optocouplers */
static void motors_force_off(void)
{
    const uint64_t mask = (1ULL << PIN_MOTOR_A1) | (1ULL << PIN_MOTOR_A2) |
                          (1ULL << PIN_MOTOR_B1) | (1ULL << PIN_MOTOR_B2);
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_MOTOR_A1, 1);
    gpio_set_level(PIN_MOTOR_A2, 1);
    gpio_set_level(PIN_MOTOR_B1, 1);
    gpio_set_level(PIN_MOTOR_B2, 1);
}

static bool ensure_wifi_ap_online(void)
{
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) == ESP_ERR_WIFI_NOT_INIT) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        if (esp_wifi_init(&cfg) != ESP_OK) return false;
    }

    esp_wifi_stop();

    wifi_config_t ap_cfg = { 0 };
    strncpy((char *)ap_cfg.ap.ssid, CAL_WIFI_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, CAL_WIFI_PASS, sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len         = (uint8_t)strlen(CAL_WIFI_SSID);
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

static const char *cap_state_str(loadcell_cap_state_t st)
{
    switch (st) {
        case LC_CAP_RUNNING: return "capturing";
        case LC_CAP_DONE:    return "done";
        default:             return "idle";
    }
}

static esp_err_t http_status_handler(httpd_req_t *req)
{
    loadcell_snapshot_t s;
    loadcell_get_snapshot(&s);

    char buf[320];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"state\":\"%s\",\"valid\":%s,\"saturated\":%s,"
             "\"captured\":%u,\"target\":%u,\"sps\":%.2f,"
             "\"total_samples\":%" PRIu32 ",\"timeouts\":%" PRIu32 ",\"sat_count\":%" PRIu32 "}",
             cap_state_str(s.cap_state),
             s.valid ? "true" : "false",
             s.saturated ? "true" : "false",
             s.cap_count, s.cap_target, s.sps,
             s.total_samples, s.timeout_count, s.sat_count);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t http_read_handler(httpd_req_t *req)
{
    loadcell_snapshot_t s;
    loadcell_get_snapshot(&s);

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"ok\":%s,\"t_ms\":%" PRIu32 ",\"raw\":%" PRId32 ","
             "\"median\":%" PRId32 ",\"filtered\":%.2f,\"std\":%.2f,"
             "\"sps\":%.2f,\"saturated\":%s}",
             s.valid ? "true" : "false",
             s.last_t_ms, s.latest_raw, s.median_raw,
             s.filtered_raw, s.std_raw, s.sps,
             s.saturated ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t http_capture_handler(httpd_req_t *req)
{
    uint16_t n = LC_DEFAULT_CAP_SAMPLES;
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "n", val, sizeof(val)) == ESP_OK) {
            int parsed = atoi(val);
            if (parsed >= 1 && parsed <= LC_MAX_CAPTURE_SAMPLES) {
                n = (uint16_t)parsed;
            }
        }
    }

    esp_err_t err = loadcell_capture_start(n);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Capture start failed");
        return ESP_FAIL;
    }

    loadcell_snapshot_t s;
    loadcell_get_snapshot(&s);
    float sps = (s.sps > 1.0f) ? s.sps : 10.0f;

    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"ok\":true,\"state\":\"capturing\",\"target\":%u,\"est_sec\":%.2f}",
             n, (float)n / sps);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

static esp_err_t http_dump_bin_handler(httpd_req_t *req)
{
    const size_t max_bytes = sizeof(loadcell_bin_header_t) +
                             (LC_MAX_CAPTURE_SAMPLES * sizeof(loadcell_sample_t));
    uint8_t *buf = malloc(max_bytes);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_ERR_NO_MEM;
    }

    loadcell_sample_t *samples = (loadcell_sample_t *)(buf + sizeof(loadcell_bin_header_t));
    uint16_t count = 0;
    uint32_t sps_x10 = 0;

    esp_err_t err = loadcell_capture_copy(samples, LC_MAX_CAPTURE_SAMPLES, &count, &sps_x10);
    if (err == ESP_ERR_INVALID_STATE) {
        free(buf);
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"capture_in_progress\"}");
        return ESP_OK;
    }

    loadcell_bin_header_t *hdr = (loadcell_bin_header_t *)buf;
    hdr->magic   = LC_BIN_MAGIC;
    hdr->version = LC_BIN_VERSION;
    hdr->count   = count;
    hdr->sps_x10 = sps_x10;

    size_t total_len = sizeof(loadcell_bin_header_t) + (count * sizeof(loadcell_sample_t));
    httpd_resp_set_type(req, "application/octet-stream");
    esp_err_t send_err = httpd_resp_send(req, (const char *)buf, (ssize_t)total_len);
    free(buf);
    return send_err;
}

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = CAL_HTTP_PORT;
    cfg.core_id     = 0;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/status",   .method = HTTP_GET, .handler = http_status_handler   },
        { .uri = "/read",     .method = HTTP_GET, .handler = http_read_handler     },
        { .uri = "/capture",  .method = HTTP_GET, .handler = http_capture_handler  },
        { .uri = "/dump.bin", .method = HTTP_GET, .handler = http_dump_bin_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    ESP_LOGI(TAG, "HTTP ready: http://192.168.4.1:%d", CAL_HTTP_PORT);
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
    /* 0. Force motors OFF immediately */
    motors_force_off();

    gpio_reset_pin(PIN_USER_LED);
    gpio_set_direction(PIN_USER_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_USER_LED, 0);

    /* 1. Keep OTA boot window */
    wifi_ap_init_base();
    ota_boot_window();

    /* 2. Bring up SoftAP + HTTP server on Core 0 */
    if (ensure_wifi_ap_online()) {
        start_http_server();
        blink_ready_led();
    }

    /* 3. Start HX711 sampling service pinned to Core 1 */
    ESP_ERROR_CHECK(loadcell_service_start());
}