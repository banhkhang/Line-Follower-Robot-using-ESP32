

/* app.c */

#include "app.h"
#include "ota.h"
#include "config.h"
#include "wifi_ap.h"
#include "telemetry.h"
#include "loadcell_service.h"
#include "line_sensor_service.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

static const char *TAG = "app";

static void system_services_init(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_lwip", ESP_LOG_WARN);

    telemetry_init();
    wifi_ap_init_base();

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
    /* 3. Normal start-up: Khởi tạo và hiệu chỉnh dãy 7 cảm biến TCRT5000 */
    if (line_sensor_service_init() != ESP_OK) {
        ESP_LOGE(TAG, "Line sensor init failed");
        return;
    }

    /* Lặp lại quy trình hiệu chỉnh bằng tay cho đến khi cả 7 mắt đều đạt chuẩn */
    while (line_sensor_service_calibrate_manual() != ESP_OK) {
        ESP_LOGW(TAG, "Thu hieu chinh lai sau 2 giay...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* Main loop: Đọc và in sai số phương ngang thô (chưa qua bộ lọc) */
    while (1) {
        line_sensor_reading_t sensor_data;

        if (line_sensor_service_read(&sensor_data) == ESP_OK) {
            if (sensor_data.line_detected) {
                ESP_LOGI(TAG, "e_y_raw = %+6.2f mm | norm=[%.0f, %.0f, %.0f, %.0f, %.0f, %.0f, %.0f]",
                         sensor_data.lateral_error_mm,
                         sensor_data.norm[0], sensor_data.norm[1], sensor_data.norm[2],
                         sensor_data.norm[3], sensor_data.norm[4], sensor_data.norm[5],
                         sensor_data.norm[6]);
            } else {
                ESP_LOGW(TAG, "LOST LINE! last e_y_raw = %+6.2f mm", sensor_data.lateral_error_mm);
            }
        } else {
            ESP_LOGE(TAG, "Line sensor read failed");
        }

        vTaskDelay(pdMS_TO_TICKS(10)); /* Chu kỳ lấy mẫu 10ms (100Hz) */
    }
}
