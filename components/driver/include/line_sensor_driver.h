#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo GPIO điều khiển LED phát (PIN_SENSOR_CTRL) và bộ ADC1 + ADC2
 *        cho 7 mắt TCRT5000 theo cấu hình trong config.h.
 */
esp_err_t line_sensor_driver_init(void);

/**
 * @brief Bật hoặc tắt dàn LED hồng ngoại phát của TCRT5000 qua PIN_SENSOR_CTRL.
 */
void line_sensor_driver_set_emitter(bool enable);

/**
 * @brief Đọc giá trị ADC 12-bit thô (0..4095) của cả 7 mắt cảm biến.
 * 
 * @param[out] raw_out Mảng chứa 7 giá trị đọc được từ PIN_SENSOR_1 đến PIN_SENSOR_7.
 * @return ESP_OK nếu đọc thành công cả 7 kênh, hoặc mã lỗi ESP-IDF (ví dụ ESP_ERR_INVALID_STATE nếu Wi-Fi đang chiếm ADC2).
 */
esp_err_t line_sensor_driver_read_raw(uint16_t raw_out[LINE_SENSOR_COUNT]);

#ifdef __cplusplus
}
#endif