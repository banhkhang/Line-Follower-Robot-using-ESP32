#include <stddef.h>
#include <math.h>
#include "loadcell_service.h"
#include "config.h"
#include "hx711.h"

static int32_t s_offset  = 0;
static bool    s_enabled = false;

#define HX711_SAT_MAX    8388607     /* full-scale glitch or overloaded cell */
#define HX711_SAT_MIN   (-8388608)
#define MAX_SAMPLES      16

static inline float net_to_grams(int32_t net)
{
    float g = (float)net / HX711_CALIBRATION_FACTOR;
#if HX711_AUTO_ABS_WEIGHT
    g = fabsf(g);
#endif
    return g;
}

/* Takes `samples` readings, discards full-scale glitches, returns trimmed mean. */
static esp_err_t read_average(int samples, int32_t *average)
{
    int32_t buf[MAX_SAMPLES];
    if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;
    if (samples < 3)           samples = 3;

    int bad = 0;
    for (int i = 0; i < samples; ) {
        int32_t s;
        esp_err_t err = hx711_read_raw(&s, LOADCELL_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }
        if (s >= HX711_SAT_MAX || s <= HX711_SAT_MIN) {
            if (++bad > samples) {
                return ESP_ERR_INVALID_RESPONSE;
            }
            continue;
        }
        buf[i++] = s;
    }

    /* Insertion sort */
    for (int i = 1; i < samples; i++) {
        int32_t key = buf[i];
        int j = i - 1;
        while (j >= 0 && buf[j] > key) {
            buf[j + 1] = buf[j];
            j--;
        }
        buf[j + 1] = key;
    }

    /* Trimmed mean: drop lowest and highest */
    int64_t sum = 0;
    for (int i = 1; i < samples - 1; i++) {
        sum += buf[i];
    }
    *average = (int32_t)(sum / (samples - 2));
    return ESP_OK;
}

esp_err_t loadcell_service_init(void)
{
    if (s_enabled) {
        return ESP_OK;
    }

    esp_err_t err = hx711_init(PIN_HX711_DOUT, PIN_HX711_SCK, HX711_GAIN);
    if (err == ESP_OK) {
        s_enabled = true;
    }
    return err;
}

void loadcell_service_deinit(void)
{
    if (!s_enabled) return;
    hx711_deinit();
    s_enabled = false;
}

bool loadcell_service_is_enabled(void)
{
    return s_enabled;
}

esp_err_t loadcell_service_tare(void)
{
    if (!s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t zero = 0;
    esp_err_t err = read_average(HX711_TARE_SAMPLES, &zero);
    if (err == ESP_OK) {
        s_offset = zero;
    }
    return err;
}

esp_err_t loadcell_service_read(loadcell_reading_t *out)
{
    if (!s_enabled)   return ESP_ERR_INVALID_STATE;
    if (out == NULL)  return ESP_ERR_INVALID_ARG;

    int32_t raw = 0;
    esp_err_t err = read_average(LOADCELL_SAMPLES, &raw);
    if (err != ESP_OK) {
        return err;
    }

    out->raw   = raw;
    out->net   = raw - s_offset;
    out->grams = net_to_grams(out->net);
    return ESP_OK;
}

esp_err_t loadcell_service_try_read_single(float *grams_out)
{
    if (!s_enabled)       return ESP_ERR_INVALID_STATE;
    if (grams_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!hx711_is_ready()) return ESP_ERR_NOT_FOUND;

    int32_t raw = 0;
    esp_err_t err = hx711_read_raw(&raw, 5);
    if (err != ESP_OK) {
        return err;
    }
    if (raw >= HX711_SAT_MAX || raw <= HX711_SAT_MIN) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *grams_out = net_to_grams(raw - s_offset);
    return ESP_OK;
}

int32_t loadcell_service_get_offset(void)
{
    return s_offset;
}
