#ifndef TCRT5000_H
#define TCRT5000_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Automatically exclude Sensor 7 (ADC2) when Wi-Fi telemetry stays active */
#if USE_TELEMETRY
    #define TCRT5000_ACTIVE_CHANNELS  6
#else
    #define TCRT5000_ACTIVE_CHANNELS  7
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

#ifdef __cplusplus
}
#endif

#endif // TCRT5000_H