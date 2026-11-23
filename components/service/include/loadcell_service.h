#pragma once

#include <stdint.h>
#include "esp_err.h"

/*
 * Load cell service: turns the HX711's raw numbers into something useful.
 *   raw   = averaged number straight from the chip
 *   net   = raw minus the "zero" captured by tare
 *   grams = net divided by LOADCELL_SCALE (see config.h)
 */
typedef struct {
    int32_t raw;
    int32_t net;
    float   grams;
} loadcell_reading_t;

/* Start the HX711 driver. */
esp_err_t loadcell_service_init(void);

void loadcell_service_deinit(void);

bool loadcell_service_is_enabled(void);

/* Measure the current (empty) load and remember it as zero. */
esp_err_t loadcell_service_tare(void);

/* Take one averaged reading. */
esp_err_t loadcell_service_read(loadcell_reading_t *out);

/* The zero value found by the last tare. */
int32_t loadcell_service_get_offset(void);
