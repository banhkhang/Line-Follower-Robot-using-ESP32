#include "sensor_service.h"
#include "tcrt5000.h"
#include "esp_log.h"

static const char *TAG = "sensor_svc";
static bool s_enabled = false;

esp_err_t sensor_service_init(void)
{
    if (s_enabled) {
        return ESP_OK;
    }

    esp_err_t err = tcrt5000_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCRT5000 driver init failed (%s)", esp_err_to_name(err));
        s_enabled = false;
        return err;
    }

    s_enabled = true;
    return ESP_OK;
}

void sensor_service_deinit(void)
{
    if (!s_enabled) {
        return;
    }
    tcrt5000_deinit();
    s_enabled = false;
}

bool sensor_service_is_enabled(void)
{
    return s_enabled;
}

esp_err_t sensor_service_read(line_sensor_reading_t *out_reading)
{
    if (!s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    out_reading->active_count = TCRT5000_ACTIVE_CHANNELS;
    return tcrt5000_read_raw(out_reading->raw);
}