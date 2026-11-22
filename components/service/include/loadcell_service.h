#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#define LC_BIN_MAGIC    0x31313758U  /* "X711" little-endian */
#define LC_BIN_VERSION  1U

typedef enum {
    LC_CAP_IDLE    = 0,
    LC_CAP_RUNNING = 1,
    LC_CAP_DONE    = 2,
} loadcell_cap_state_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int32_t  raw;
} loadcell_sample_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    uint32_t sps_x10;
} loadcell_bin_header_t;

typedef struct {
    int32_t              latest_raw;
    int32_t              median_raw;
    float                filtered_raw;   /* MAD-rejected window mean */
    float                std_raw;
    float                sps;
    uint32_t             last_t_ms;
    uint32_t             total_samples;
    uint32_t             timeout_count;
    uint32_t             sat_count;
    bool                 valid;
    bool                 saturated;
    loadcell_cap_state_t cap_state;
    uint16_t             cap_count;
    uint16_t             cap_target;
} loadcell_snapshot_t;

esp_err_t loadcell_service_start(void);
void      loadcell_get_snapshot(loadcell_snapshot_t *out);
esp_err_t loadcell_capture_start(uint16_t target_samples);
esp_err_t loadcell_capture_copy(loadcell_sample_t *dst,
                                uint16_t max_samples,
                                uint16_t *out_count,
                                uint32_t *out_sps_x10);