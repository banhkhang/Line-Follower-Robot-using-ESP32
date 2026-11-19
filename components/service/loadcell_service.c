#include <stddef.h>
#include "loadcell_service.h"
#include "config.h"
#include "hx711.h"

static int32_t s_offset  = 0;
static bool    s_enabled = false;

static esp_err_t read_average(int samples, int32_t *average)
{
    int64_t sum = 0;

    for (int i = 0; i < samples; i++) {
        int32_t sample;
        esp_err_t err = hx711_read_raw(&sample, LOADCELL_TIMEOUT_MS);
        if (err != ESP_OK) {
            return err;
        }
        sum += sample;
    }

    *average = (int32_t)(sum / samples);
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
    if (!s_enabled) {
        return;
    }
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

    int32_t zero;
    esp_err_t err = read_average(HX711_TARE_SAMPLES, &zero);
    if (err == ESP_OK) {
        s_offset = zero;
    }
    return err;
}

esp_err_t loadcell_service_read(loadcell_reading_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t raw;
    esp_err_t err = read_average(LOADCELL_SAMPLES, &raw);
    if (err != ESP_OK) {
        return err;
    }

    out->raw   = raw;
    out->net   = raw - s_offset;
    out->grams = (float)out->net / HX711_CALIBRATION_FACTOR;
    return ESP_OK;
}

int32_t loadcell_service_get_offset(void)
{
    return s_offset;
}