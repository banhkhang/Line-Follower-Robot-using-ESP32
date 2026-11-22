#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HX711_GAIN_A_128 = 1,
    HX711_GAIN_B_32  = 2,
    HX711_GAIN_A_64  = 3,
} hx711_gain_t;

typedef struct {
    gpio_num_t   dout;
    gpio_num_t   sck;
    hx711_gain_t gain;
} hx711_t;

esp_err_t hx711_init(hx711_t *dev, gpio_num_t dout, gpio_num_t sck, hx711_gain_t gain);
bool      hx711_is_ready(const hx711_t *dev);
esp_err_t hx711_read_raw(const hx711_t *dev, int32_t *out_raw, uint32_t timeout_ms);
bool      hx711_is_saturated(int32_t raw);