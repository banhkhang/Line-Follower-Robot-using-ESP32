#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int   raw[LINE_SENSOR_MAX_COUNT];         /* Raw 12-bit ADC (0-4095) */
    float norm[LINE_SENSOR_MAX_COUNT];        /* Normalized 0.0 (white) to 100.0 (black) */
    float position_mm;                        /* Weighted average error e_line (mm) */
    bool  line_detected;                      /* True if total normalized signal > threshold */
    uint8_t active_count;                     /* 6 if USE_TELEMETRY == 1, else 7 */
} line_sensor_reading_t;

esp_err_t sensor_service_init(void);
void      sensor_service_deinit(void);
bool      sensor_service_is_enabled(void);
/**
 * @brief Sweep the sensor bar across white floor and black line for `duration_ms`
 *        to automatically record R_white[i] and R_black[i] for each sensor.
 */
esp_err_t sensor_service_calibrate_sweep(void);

esp_err_t sensor_service_read(line_sensor_reading_t *out_reading);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_SERVICE_H