#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

esp_err_t hx711_init(gpio_num_t dout_pin, gpio_num_t sck_pin, int gain);
void      hx711_deinit(void);
bool      hx711_is_ready(void);
esp_err_t hx711_read_raw(int32_t *value, uint32_t timeout_ms);