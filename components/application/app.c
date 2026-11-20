

/* app.c */

#include "app.h"
#include "ota.h"
#include "wifi_ap.h"
#include "telemetry.h"
#include "loadcell_service.h"
#include "line_sensor_service.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

static const char *TAG = "app";

void app_start(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    ESP_LOGW("BOOT", "================================");
    ESP_LOGW("BOOT", "ESP32 RESTART DETECTED");
    ESP_LOGW("BOOT", "Reset reason: %d", reason);
    ESP_LOGW("BOOT", "================================");
    
    /* 1. Start log capture immediately (queues logs until Wi-Fi is up) */
    wifi_ap_init_base();          /* 1. Start lwIP + event loop once */
    telemetry_init();   

    ESP_LOGI(TAG, "Starting up");
    
    /* 2. Open OTA window (turns on LineRobot-OTA; boot logs broadcast here!) */
    ota_boot_window();

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