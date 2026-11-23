#include "tcrt5000.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "tcrt5000";

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
#if !USE_TELEMETRY
static adc_oneshot_unit_handle_t s_adc2_handle = NULL;
#endif

typedef struct {
    adc_unit_t unit;
    adc_channel_t channel;
    int gpio_num;
} tcrt_channel_map_t;

static const tcrt_channel_map_t s_sensor_map[TCRT5000_ACTIVE_CHANNELS] = {
    { ADC_UNIT_1, ADC_CHANNEL_0, PIN_SENSOR_1 }, // S1: GPIO 36
    { ADC_UNIT_1, ADC_CHANNEL_3, PIN_SENSOR_2 }, // S2: GPIO 39
    { ADC_UNIT_1, ADC_CHANNEL_6, PIN_SENSOR_3 }, // S3: GPIO 34
    { ADC_UNIT_1, ADC_CHANNEL_7, PIN_SENSOR_4 }, // S4: GPIO 35
    { ADC_UNIT_1, ADC_CHANNEL_4, PIN_SENSOR_5 }, // S5: GPIO 32
    { ADC_UNIT_1, ADC_CHANNEL_5, PIN_SENSOR_6 }, // S6: GPIO 33
#if !USE_TELEMETRY
    { ADC_UNIT_2, ADC_CHANNEL_8, PIN_SENSOR_7 }, // S7: GPIO 25 (Only compiled when Wi-Fi is off)
#endif
};

void tcrt5000_set_emitters(bool enable)
{
    gpio_set_level(PIN_SENSOR_CTRL, enable ? 1 : 0);
}

esp_err_t tcrt5000_init(void)
{
    esp_err_t ret;

    /* 1. Configure low-side N-MOSFET control pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_SENSOR_CTRL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CTRL GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Keep IR emitters ON for testing */
    tcrt5000_set_emitters(true);

    /* 2. Initialize ADC1 */
    adc_oneshot_unit_init_cfg_t init_cfg1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_cfg1, &s_adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 3. Initialize ADC2 only if Wi-Fi telemetry is disabled */
#if !USE_TELEMETRY
    adc_oneshot_unit_init_cfg_t init_cfg2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ret = adc_oneshot_new_unit(&init_cfg2, &s_adc2_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC2 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
#else
    ESP_LOGW(TAG, "USE_TELEMETRY=1: Sensor 7 (GPIO %d / ADC2) disabled to prevent Wi-Fi conflict", PIN_SENSOR_7);
#endif

    /* 4. Configure active channels */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };

    for (int i = 0; i < TCRT5000_ACTIVE_CHANNELS; i++) {
        adc_oneshot_unit_handle_t handle = s_adc1_handle;
#if !USE_TELEMETRY
        if (s_sensor_map[i].unit == ADC_UNIT_2) {
            handle = s_adc2_handle;
        }
#endif
        ret = adc_oneshot_config_channel(handle, s_sensor_map[i].channel, &chan_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to config S%d (GPIO %d): %s",
                     i + 1, s_sensor_map[i].gpio_num, esp_err_to_name(ret));
            return ret;
        }
    }

    ESP_LOGI(TAG, "Initialized %d/%d TCRT5000 channels (IR emitters ON)",
             TCRT5000_ACTIVE_CHANNELS, LINE_SENSOR_MAX_COUNT);
    return ESP_OK;
}

esp_err_t tcrt5000_deinit(void)
{
    tcrt5000_set_emitters(false);

    if (s_adc1_handle) {
        adc_oneshot_del_unit(s_adc1_handle);
        s_adc1_handle = NULL;
    }
#if !USE_TELEMETRY
    if (s_adc2_handle) {
        adc_oneshot_del_unit(s_adc2_handle);
        s_adc2_handle = NULL;
    }
#endif
    return ESP_OK;
}

esp_err_t tcrt5000_read_raw(int raw_values[LINE_SENSOR_MAX_COUNT])
{
    if (raw_values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Default all slots to -1 (marks excluded sensors clearly) */
    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        raw_values[i] = -1;
    }

    esp_err_t overall_ret = ESP_OK;

    for (int i = 0; i < TCRT5000_ACTIVE_CHANNELS; i++) {
        adc_oneshot_unit_handle_t handle = s_adc1_handle;
#if !USE_TELEMETRY
        if (s_sensor_map[i].unit == ADC_UNIT_2) {
            handle = s_adc2_handle;
        }
#endif
        esp_err_t ret = adc_oneshot_read(handle, s_sensor_map[i].channel, &raw_values[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Read error on S%d (GPIO %d): %s",
                     i + 1, s_sensor_map[i].gpio_num, esp_err_to_name(ret));
            raw_values[i] = -1;
            overall_ret = ret;
        }
    }

    return overall_ret;
}