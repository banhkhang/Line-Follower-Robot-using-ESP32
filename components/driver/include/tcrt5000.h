#ifndef TCRT5000_H
#define TCRT5000_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

#define LINE_SENSOR_MAX_COUNT  7

#if USE_TELEMETRY
#define TCRT5000_ACTIVE_CHANNELS 5   /* Symmetric center window: S2, S3, S4, S5, S6 */
#else
#define TCRT5000_ACTIVE_CHANNELS 7   /* Full array: S1..S7 */
#endif
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize GPIO26 (IR emitter MOSFET) and active ADC units/channels.
 *        When USE_TELEMETRY == 1, ADC2 and Sensor 7 are excluded.
 */
esp_err_t tcrt5000_init(void);

/**
 * @brief Deinitialize ADC units and turn off IR emitters.
 */
esp_err_t tcrt5000_deinit(void);

/**
 * @brief Enable or disable the IR emitter LEDs via the low-side MOSFET.
 */
void tcrt5000_set_emitters(bool enable);

/**
 * @brief Read raw 12-bit analog values (0-4095) for all active channels.
 *        If USE_TELEMETRY == 1, raw_values[6] is set to -1.
 *
 * @param[out] raw_values Array of size LINE_SENSOR_MAX_COUNT (7).
 */
esp_err_t tcrt5000_read_raw(int raw_values[LINE_SENSOR_MAX_COUNT]);
esp_err_t tcrt5000_read_compensated(int net_values[LINE_SENSOR_MAX_COUNT],
                                    int amb_values[LINE_SENSOR_MAX_COUNT]);
#ifdef __cplusplus
}
#endif

#endif // TCRT5000_H