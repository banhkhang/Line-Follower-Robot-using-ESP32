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

esp_err_t loadcell_service_tare(void);
esp_err_t loadcell_service_read(loadcell_reading_t *out);
int32_t   loadcell_service_get_offset(void);