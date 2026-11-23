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
    int raw[LINE_SENSOR_MAX_COUNT]; /* 0-4095 for active channels, -1 if disabled/error */
    uint8_t active_count;           /* 6 when USE_TELEMETRY == 1, 7 when USE_TELEMETRY == 0 */
} line_sensor_reading_t;

esp_err_t sensor_service_init(void);
void      sensor_service_deinit(void);
bool      sensor_service_is_enabled(void);
esp_err_t sensor_service_read(line_sensor_reading_t *out_reading);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_SERVICE_H