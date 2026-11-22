#include "loadcell_service.h"
#include "config.h"
#include "hx711.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "lc_svc";

static hx711_t             s_hx;
static SemaphoreHandle_t   s_lock = NULL;

static int32_t             s_ring[LC_FILTER_WIN];
static uint8_t             s_ring_idx   = 0;
static uint8_t             s_ring_count = 0;

static loadcell_sample_t   s_cap_buf[LC_MAX_CAPTURE_SAMPLES];
static loadcell_snapshot_t s_snap;

static void sort_i32(int32_t *arr, uint8_t n)
{
    for (uint8_t i = 1; i < n; i++) {
        int32_t key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

static void compute_mad_stats(const int32_t *ring, uint8_t n,
                              int32_t *out_median,
                              float *out_mean,
                              float *out_std)
{
    int32_t sorted[LC_FILTER_WIN];
    int32_t devs[LC_FILTER_WIN];

    memcpy(sorted, ring, n * sizeof(int32_t));
    sort_i32(sorted, n);
    int32_t median = sorted[n / 2];

    for (uint8_t i = 0; i < n; i++) {
        devs[i] = abs(sorted[i] - median);
    }
    sort_i32(devs, n);
    int32_t mad = devs[n / 2];

    /* 3.5 * 1.4826 * MAD with a 32-count floor to preserve normal LSB noise */
    float thresh = 5.189f * (float)mad;
    if (thresh < 32.0f) thresh = 32.0f;

    double sum = 0.0;
    double sum_sq = 0.0;
    int kept = 0;

    for (uint8_t i = 0; i < n; i++) {
        float diff = fabsf((float)(ring[i] - median));
        if (diff <= thresh) {
            double v = (double)ring[i];
            sum += v;
            sum_sq += v * v;
            kept++;
        }
    }

    if (kept == 0) {
        *out_median = median;
        *out_mean   = (float)median;
        *out_std    = 0.0f;
        return;
    }

    double mean = sum / (double)kept;
    double var  = (kept > 1) ? ((sum_sq - (double)kept * mean * mean) / (double)(kept - 1)) : 0.0;
    if (var < 0.0) var = 0.0;

    *out_median = median;
    *out_mean   = (float)mean;
    *out_std    = (float)sqrt(var);
}

static void loadcell_task(void *arg)
{
    (void)arg;
    int64_t prev_us = 0;

    ESP_LOGI(TAG, "Core %d HX711 task started (DOUT=%d, SCK=%d)",
             xPortGetCoreID(), PIN_HX711_DOUT, PIN_HX711_SCK);

    while (1) {
        int32_t raw = 0;
        esp_err_t err = hx711_read_raw(&s_hx, &raw, HX711_READY_TIMEOUT_MS);
        int64_t now_us = esp_timer_get_time();
        uint32_t now_ms = (uint32_t)(now_us / 1000LL);

        xSemaphoreTake(s_lock, portMAX_DELAY);

        if (err != ESP_OK) {
            s_snap.timeout_count++;
            s_snap.valid = false;
            xSemaphoreGive(s_lock);
            continue;
        }

        if (prev_us > 0) {
            float dt_s = (float)(now_us - prev_us) * 1e-6f;
            if (dt_s > 0.002f && dt_s < 1.0f) {
                float inst_sps = 1.0f / dt_s;
                s_snap.sps = (s_snap.sps <= 0.1f) ? inst_sps : (0.85f * s_snap.sps + 0.15f * inst_sps);
            }
        }
        prev_us = now_us;

        bool sat = hx711_is_saturated(raw);
        if (sat) {
            s_snap.sat_count++;
        }

        s_snap.latest_raw = raw;
        s_snap.last_t_ms  = now_ms;
        s_snap.total_samples++;
        s_snap.valid     = true;
        s_snap.saturated = sat;

        /* Update rolling live filter */
        s_ring[s_ring_idx] = raw;
        s_ring_idx = (uint8_t)((s_ring_idx + 1) % LC_FILTER_WIN);
        if (s_ring_count < LC_FILTER_WIN) s_ring_count++;

        compute_mad_stats(s_ring, s_ring_count,
                          &s_snap.median_raw,
                          &s_snap.filtered_raw,
                          &s_snap.std_raw);

        /* Record unfiltered raw sample if capture is active */
        if (s_snap.cap_state == LC_CAP_RUNNING) {
            if (s_snap.cap_count < s_snap.cap_target &&
                s_snap.cap_count < LC_MAX_CAPTURE_SAMPLES) {
                s_cap_buf[s_snap.cap_count].t_ms = now_ms;
                s_cap_buf[s_snap.cap_count].raw  = raw;
                s_snap.cap_count++;
            }
            if (s_snap.cap_count >= s_snap.cap_target) {
                s_snap.cap_state = LC_CAP_DONE;
            }
        }

        xSemaphoreGive(s_lock);
    }
}

esp_err_t loadcell_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;

    memset(&s_snap, 0, sizeof(s_snap));
    esp_err_t err = hx711_init(&s_hx,
                               (gpio_num_t)PIN_HX711_DOUT,
                               (gpio_num_t)PIN_HX711_SCK,
                               (hx711_gain_t)HX711_GAIN_PULSES);
    if (err != ESP_OK) return err;

    BaseType_t ok = xTaskCreatePinnedToCore(
        loadcell_task, "hx711_task",
        LC_TASK_STACK, NULL, LC_TASK_PRIO, NULL, LC_TASK_CORE
    );
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

void loadcell_get_snapshot(loadcell_snapshot_t *out)
{
    if (!out || !s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_snap;
    xSemaphoreGive(s_lock);
}

esp_err_t loadcell_capture_start(uint16_t target_samples)
{
    if (!s_lock) return ESP_ERR_INVALID_STATE;
    if (target_samples == 0) target_samples = LC_DEFAULT_CAP_SAMPLES;
    if (target_samples > LC_MAX_CAPTURE_SAMPLES) target_samples = LC_MAX_CAPTURE_SAMPLES;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_snap.cap_count  = 0;
    s_snap.cap_target = target_samples;
    s_snap.cap_state  = LC_CAP_RUNNING;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t loadcell_capture_copy(loadcell_sample_t *dst,
                                uint16_t max_samples,
                                uint16_t *out_count,
                                uint32_t *out_sps_x10)
{
    if (!dst || !out_count || !s_lock) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_snap.cap_state == LC_CAP_RUNNING) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t n = s_snap.cap_count;
    if (n > max_samples) n = max_samples;
    if (n > 0) {
        memcpy(dst, s_cap_buf, n * sizeof(loadcell_sample_t));
    }
    *out_count = n;
    if (out_sps_x10) {
        *out_sps_x10 = (uint32_t)(s_snap.sps * 10.0f + 0.5f);
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}