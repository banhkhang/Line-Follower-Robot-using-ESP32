/*
 * ============================================================================
 * data_logger.c - Bo dem log nhi phan theo khoi (Chunked SRAM) cho He B
 * ============================================================================
 */

#include "data_logger.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "data_log";

#define LOG_CHUNK_SAMPLES       200U        /* 200 * 52 B = 10,400 bytes / chunk */
#define LOG_MAX_CHUNKS          16U         /* Toi da 3,200 mau neu du RAM */
#define WIFI_HEAP_RESERVE_BYTES (88U * 1024U) /* Giu toi thieu 88 KB DRAM cho Wi-Fi Burst */

static robot_log_sample_t *s_chunks[LOG_MAX_CHUNKS] = {0};
static uint32_t            s_num_chunks = 0;
static uint32_t            s_capacity   = 0;
static uint32_t            s_count      = 0;
static robot_run_summary_t s_summary    = {0};

esp_err_t data_logger_prepare(uint32_t max_samples)
{
    data_logger_free();
    memset(&s_summary, 0, sizeof(s_summary));

    uint32_t wanted_chunks = (max_samples + LOG_CHUNK_SAMPLES - 1U) / LOG_CHUNK_SAMPLES;
    if (wanted_chunks > LOG_MAX_CHUNKS) {
        wanted_chunks = LOG_MAX_CHUNKS;
    }

    const size_t chunk_bytes = (size_t)LOG_CHUNK_SAMPLES * sizeof(robot_log_sample_t);

    for (uint32_t c = 0; c < wanted_chunks; c++) {
        size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        if (free_dram <= (WIFI_HEAP_RESERVE_BYTES + chunk_bytes)) {
            ESP_LOGW(TAG, "Stop allocating at %lu samples to preserve %u KB DRAM for Wi-Fi burst",
                     (unsigned long)s_capacity, (unsigned)(WIFI_HEAP_RESERVE_BYTES / 1024U));
            break;
        }

        robot_log_sample_t *blk = (robot_log_sample_t *)heap_caps_malloc(
            chunk_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        if (blk == NULL) {
            break;
        }
        s_chunks[s_num_chunks++] = blk;
        s_capacity += LOG_CHUNK_SAMPLES;
    }

    if (s_capacity == 0) {
        ESP_LOGE(TAG, "Failed to allocate any SRAM log buffer!");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "SRAM logger ready: %lu samples (%lu KB in %lu chunks), free DRAM left: %lu KB",
             (unsigned long)s_capacity,
             (unsigned long)((s_capacity * sizeof(robot_log_sample_t)) / 1024U),
             (unsigned long)s_num_chunks,
             (unsigned long)(heap_caps_get_free_size(MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL) / 1024U));
    return ESP_OK;
}

void data_logger_free(void)
{
    for (uint32_t c = 0; c < s_num_chunks; c++) {
        if (s_chunks[c] != NULL) {
            free(s_chunks[c]);
            s_chunks[c] = NULL;
        }
    }
    s_num_chunks = 0;
    s_capacity   = 0;
    s_count      = 0;
}

bool data_logger_push(const robot_log_sample_t *sample)
{
    if (sample == NULL || s_count >= s_capacity) {
        return false;
    }
    uint32_t chunk_idx = s_count / LOG_CHUNK_SAMPLES;
    uint32_t slot_idx  = s_count % LOG_CHUNK_SAMPLES;
    s_chunks[chunk_idx][slot_idx] = *sample;
    s_count++;
    return true;
}

bool data_logger_get(uint32_t index, robot_log_sample_t *out)
{
    if (out == NULL || index >= s_count) {
        return false;
    }
    uint32_t chunk_idx = index / LOG_CHUNK_SAMPLES;
    uint32_t slot_idx  = index % LOG_CHUNK_SAMPLES;
    *out = s_chunks[chunk_idx][slot_idx];
    return true;
}

uint32_t data_logger_count(void)
{
    return s_count;
}

uint32_t data_logger_capacity(void)
{
    return s_capacity;
}

bool data_logger_is_full(void)
{
    return (s_capacity > 0 && s_count >= s_capacity);
}

void data_logger_set_summary(const robot_run_summary_t *summary)
{
    if (summary != NULL) {
        s_summary = *summary;
    }
}

void data_logger_get_summary(robot_run_summary_t *out)
{
    if (out != NULL) {
        *out = s_summary;
    }
}
