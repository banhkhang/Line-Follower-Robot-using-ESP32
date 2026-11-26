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
    int   min[LINE_SENSOR_MAX_COUNT];     /* Lowest ambient reading seen (brightest room light) */
    int   max[LINE_SENSOR_MAX_COUNT];     /* Highest ambient reading seen (darkest shadow) */
    float avg[LINE_SENSOR_MAX_COUNT];     /* Running average ambient ADC across samples */
    int   current[LINE_SENSOR_MAX_COUNT]; /* Latest single-shot ambient reading (val_off) */
} line_sensor_amb_t;

typedef struct {
    int               raw[LINE_SENSOR_MAX_COUNT];  /* Ambient-compensated net ADC (val_off - val_on) */
    float             norm[LINE_SENSOR_MAX_COUNT]; /* Normalized 0.0 (white) to 100.0 (black) */
    line_sensor_amb_t amb;                         /* Ambient light statistics (IR emitters OFF) */
    float             position_mm;                 /* Weighted average error e_line (mm) */
    bool              line_detected;               /* True if total normalized signal > threshold */
    uint8_t           active_count;                /* TCRT5000_ACTIVE_CHANNELS (5 or 7) */
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