#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*
 * HX711 24-bit load cell amplifier driver.
 *
 * Talks to the chip with two wires (DOUT, SCK) using plain GPIO.
 * Supports Channel A gain 128 or 64. The driver returns raw signed counts;
 * gram conversion and tare live in loadcell_service.
 */

/* Set up the two pins and gain (128 or 64). Call once at start-up. */
esp_err_t hx711_init(gpio_num_t dout_pin, gpio_num_t sck_pin, int gain);
void      hx711_deinit(void);

/* true when a new sample is waiting (DOUT is LOW). */
bool      hx711_is_ready(void);

/*
 * Wait for a new sample and read it.
 * value      : signed raw reading (-8388608 .. 8388607)
 * timeout_ms : how long to wait for the chip
 * Returns ESP_OK, ESP_ERR_TIMEOUT, or ESP_ERR_INVALID_STATE.
 */
esp_err_t hx711_read_raw(int32_t *value, uint32_t timeout_ms);
