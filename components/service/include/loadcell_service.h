#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    int32_t raw;
    int32_t net;
    float   grams;
} loadcell_reading_t;

esp_err_t loadcell_service_init(void);
void      loadcell_service_deinit(void);
bool      loadcell_service_is_enabled(void);

/* Measure the current (empty) load and remember it as zero. */
esp_err_t loadcell_service_tare(void);
esp_err_t loadcell_service_read(loadcell_reading_t *out);

/*
 * Non-blocking single-sample read (returns ESP_ERR_NOT_FOUND if HX711 DOUT is not ready yet).
 * Ideal for real-time loading-station detection without stalling the outer-loop task.
 */
esp_err_t loadcell_service_try_read_single(float *grams_out);

int32_t   loadcell_service_get_offset(void);
