/* app.c */

#include "app.h"
#include "ota.h"
#include "config.h"
#include "wifi_ap.h"
#include "telemetry.h"
#include "loadcell_service.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

static void system_services_init(void)
{
    wifi_ap_init_base();
    telemetry_init();
    ESP_LOGI(TAG, "System telemetry ready");

    /* Opens SoftAP + HTTP upload page; keeps SoftAP alive if USE_TELEMETRY == 1 */
    ESP_LOGI(TAG, "Starting up");
    ota_boot_window();
}

static esp_err_t sensors_start(void)
{
    esp_err_t err = loadcell_service_init();
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

    if (sensors_start() != ESP_OK) {
        ESP_LOGW(TAG, "Running without load cell service");
    }

    while (1) {
        if (loadcell_service_is_enabled()) {
            loadcell_reading_t r;
            if (loadcell_service_read(&r) == ESP_OK) {
                ESP_LOGI(TAG, "raw=%ld  net=%ld  grams=%.2f",
                         (long)r.raw, (long)r.net, r.grams);
            } else {
                ESP_LOGW(TAG, "Load cell read failed");
            }
        }

        /* Future: motor_service_update(), line_follower_step(), etc. */
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
}
