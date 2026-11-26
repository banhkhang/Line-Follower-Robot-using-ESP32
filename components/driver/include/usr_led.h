#ifndef USR_LED_H
#define USR_LED_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Reset and configure PIN_USER_LED as a push-pull digital output (initially OFF).
 */
esp_err_t usr_led_init(void);

/**
 * @brief Turn the LED solid ON or OFF (stops any active blink/rhythm pattern).
 */
void usr_led_set(bool on);

/**
 * @brief Non-blocking square-wave blink update using FreeRTOS tick time.
 *        Call inside polling loops (e.g., OTA window or calibration sweep).
 */
void usr_led_blink_poll(uint32_t half_period_ms);

/**
 * @brief Start autonomous background square-wave blinking (half_period_ms ON / OFF).
 *        Pass 0 to stop and turn OFF.
 */
void usr_led_start_blink(uint32_t half_period_ms);

/**
 * @brief Start the autonomous 1 - 2 - 1 - 1 - 2 operational heartbeat rhythm:
 *        - '1' beat: 0.1s ON, 0.9s OFF (1.0s)
 *        - '2' beat: 0.1s ON, 0.1s OFF, 0.1s ON, 0.2s OFF (0.5s)
 */
void usr_led_start_run_pattern(void);

#ifdef __cplusplus
}
#endif

#endif // USR_LED_H