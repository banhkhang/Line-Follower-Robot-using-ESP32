#include <stdbool.h>
#include <string.h>
#include "ota.h"
#include "wifi_ap.h"
#include "config.h"
#include "usr_led.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ota";
static volatile bool s_busy = false;

static const char PAGE[] =
"<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>LineRobot ESP32 OTA</title></head>"
"<body style='font-family:sans-serif;max-width:420px;margin:2em auto'>"
"<h2>LineRobot ESP32 Firmware Update</h2>"
"<input type='file' id='f' accept='.bin'><br><br>"
"<button onclick='up()'>Upload</button><p id='s'></p>"
"<script>"
"function up(){var f=document.getElementById('f').files[0],s=document.getElementById('s');"
"if(!f){s.innerText='Choose a .bin file first';return;}"
"var x=new XMLHttpRequest();x.open('POST','/update');"
"x.upload.onprogress=function(e){if(e.lengthComputable)s.innerText='Uploading '+Math.round(e.loaded*100/e.total)+'%';};"
"x.onload=function(){s.innerText=x.responseText;};"
"x.onerror=function(){s.innerText='Connection lost (ESP32 may be rebooting)';};"
"x.send(f);}"
"</script></body></html>";

static esp_err_t page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t fail(httpd_req_t *req, esp_ota_handle_t handle, const char *msg)
{
    ESP_LOGE(TAG, "%s", msg);
    if (handle) {
        esp_ota_abort(handle);
    }
    s_busy = false;
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, msg);
    return ESP_FAIL;
}

static esp_err_t update_handler(httpd_req_t *req)
{
    const esp_partition_t *slot = esp_ota_get_next_update_partition(NULL);
    if (slot == NULL) {
        return fail(req, 0, "No spare OTA partition - check partitions.csv");
    }
    if (req->content_len == 0) {
        return fail(req, 0, "Empty file");
    }

    s_busy = true;

    esp_ota_handle_t handle = 0;
    if (esp_ota_begin(slot, req->content_len, &handle) != ESP_OK) {
        return fail(req, 0, "Could not start OTA update");
    }

    char buf[1024];
    int remaining = (int)req->content_len;

    while (remaining > 0) {
        int want = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
        int n = httpd_req_recv(req, buf, want);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            return fail(req, handle, "Upload interrupted");
        }
        if (esp_ota_write(handle, buf, n) != ESP_OK) {
            return fail(req, handle, "Flash write failed");
        }
        remaining -= n;
    }

    if (esp_ota_end(handle) != ESP_OK) {
        return fail(req, 0, "Invalid firmware image");
    }
    if (esp_ota_set_boot_partition(slot) != ESP_OK) {
        return fail(req, 0, "Could not set boot partition");
    }

    ESP_LOGI(TAG, "OTA update OK, rebooting...");
    httpd_resp_sendstr(req, "Done! ESP32 is rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

void ota_boot_window(void)
{
    /* 1. Configure BOOT button GPIO */
    gpio_reset_pin(PIN_BOOT_BUTTON);
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << PIN_BOOT_BUTTON,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    /* 2. Start Wi-Fi SoftAP */
    if (wifi_ap_on(OTA_WIFI_SSID, OTA_WIFI_PASS) != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi AP failed to start, skipping OTA window");
        return;
    }

    httpd_handle_t server = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;

    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed to start, skipping OTA window");
#if !KEEP_WIFI_AP_AFTER_OTA
        wifi_ap_off();
#endif
        return;
    }

    httpd_uri_t page   = { .uri = "/",       .method = HTTP_GET,  .handler = page_handler };
    httpd_uri_t update = { .uri = "/update", .method = HTTP_POST, .handler = update_handler };
    httpd_register_uri_handler(server, &page);
    httpd_register_uri_handler(server, &update);

    ESP_LOGI(TAG, "OTA window open (%d s) | SSID: %s | URL: http://192.168.4.1 | Press BOOT to skip",
             OTA_WINDOW_MS / 1000, OTA_WIFI_SSID);

    TickType_t start_tick   = xTaskGetTickCount();
    TickType_t window_ticks = pdMS_TO_TICKS(OTA_WINDOW_MS);
    int last_remaining      = -1;

    while (1) {
        TickType_t elapsed_ticks   = xTaskGetTickCount() - start_tick;
        TickType_t remaining_ticks = (elapsed_ticks < window_ticks) ? (window_ticks - elapsed_ticks) : 0;
        uint32_t remaining_ms      = (uint32_t)remaining_ticks * portTICK_PERIOD_MS;
        int remaining_sec          = (int)((remaining_ms + 999) / 1000);

        if (remaining_sec != last_remaining) {
            ESP_LOGI(TAG, "OTA | %02d s remaining (http://192.168.4.1)", remaining_sec);
            last_remaining = remaining_sec;
        }

        if (remaining_ticks > 0 && !s_busy && gpio_get_level(PIN_BOOT_BUTTON) == 0) {
            ESP_LOGI(TAG, "BOOT pressed -> skipping OTA window");
            /* Wait until button is released so it doesn't bleed into IDLE mode */
            while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            break;
        }

        if (remaining_ticks == 0 && !s_busy) {
            ESP_LOGI(TAG, "OTA window expired");
            break;
        }

        /* LED1 heartbeat: blink every 1000 ms */
        usr_led_blink_poll(1000);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    usr_led_set(false);
    httpd_stop(server);

#if !KEEP_WIFI_AP_AFTER_OTA
    wifi_ap_off();
    ESP_LOGI(TAG, "OTA closed, Wi-Fi AP stopped");
#else
    ESP_LOGI(TAG, "OTA closed, keeping SoftAP '%s' active for MATLAB link", OTA_WIFI_SSID);
#endif
}
