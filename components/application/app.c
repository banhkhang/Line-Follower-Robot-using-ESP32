

/* app.c */

#include "app.h"
#include "ota.h"
#include "config.h"
#include "wifi_ap.h"
#include "telemetry.h"
#include "loadcell_service.h"
#include "sensor_service.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

static const char *TAG = "app";

static void system_services_init(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_lwip", ESP_LOG_WARN);

    /* 1. Initialize NVS, esp_netif, and default event loop FIRST */
    wifi_ap_init_base();

    /* 2. Now LwIP's tcpip mbox exists—safe to initialize UDP telemetry */
    telemetry_init();

    ESP_LOGI(TAG, "System telemetry ready");

    /* Stage 1: OTA Boot Window (press BOOT to skip) */
    ESP_LOGI(TAG, "Starting up");
    ota_boot_window();
}

static esp_err_t sensors_start(void)
{
    /* Stage 2: Line Sensor Init & Interactive Sweep Calibration */
    esp_err_t err = sensor_service_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Running without line sensor service (%s)", esp_err_to_name(err));
    }
    else {
        sensor_service_calibrate_sweep();
    }

    /* Stage 3: Load Cell (HX711) Init & Tare */
    err = loadcell_service_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Load cell init failed (%s)", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Taring in 2 s - keep the load cell EMPTY");
    vTaskDelay(pdMS_TO_TICKS(2000));

    err = loadcell_service_tare();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Tare failed (%s) - check DOUT/SCK wiring and 3V3 power",
                 esp_err_to_name(err));
        loadcell_service_deinit();
        return err;
    }

    ESP_LOGI(TAG, "Tare done, offset = %ld", (long)loadcell_service_get_offset());
    return ESP_OK;
}

void app_start(void)
{
    system_services_init();
    sensors_start();

    uint8_t loadcell_div = 0;

    while (1) {
        /* 1. Line Sensor Log (every 200 ms) */
        if (sensor_service_is_enabled()) {
            line_sensor_reading_t line;
            if (sensor_service_read(&line) == ESP_OK) {
                /* Build quick 6/7-char visual bar: '#' >= 50%, '+' >= 15%, '.' < 15% */
                char bar[LINE_SENSOR_MAX_COUNT + 1] = {0};
                for (int i = 0; i < line.active_count; i++) {
                    if (line.norm[i] >= 50.0f)      bar[i] = '#';
                    else if (line.norm[i] >= 15.0f) bar[i] = '+';
                    else                            bar[i] = '.';
                }
#if USE_TELEMETRY
                ESP_LOGI(TAG, "LINE [%s] | %3.0f %3.0f %3.0f %3.0f %3.0f %3.0f | err=%+6.2f mm | %s",
                         bar,
                         line.norm[0], line.norm[1], line.norm[2],
                         line.norm[3], line.norm[4], line.norm[5],
                         line.position_mm,
                         line.line_detected ? "OK" : "LOST");
#else
                ESP_LOGI(TAG, "LINE [%s] | %3.0f %3.0f %3.0f %3.0f %3.0f %3.0f %3.0f | err=%+6.2f mm | %s",
                         bar,
                         line.norm[0], line.norm[1], line.norm[2],
                         line.norm[3], line.norm[4], line.norm[5], line.norm[6],
                         line.position_mm,
                         line.line_detected ? "OK" : "LOST");
#endif
            } else {
                ESP_LOGW(TAG, "Line sensor read failed");
            }
        }

        /* 2. Load Cell Log (once every 10 sensor logs = every 2.0 s) */
        if (++loadcell_div >= 10) {
            loadcell_div = 0;
            if (loadcell_service_is_enabled()) {
                loadcell_reading_t r;
                if (loadcell_service_read(&r) == ESP_OK) {
                    ESP_LOGI(TAG, "WEIGHT     | raw=%8ld | net=%8ld | %7.2f g",
                             (long)r.raw, (long)r.net, r.grams);
                } else {
                    ESP_LOGW(TAG, "Load cell read failed");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}