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
    adc_unit_t    unit;
    adc_channel_t channel;
    int           gpio_num;
    int           sensor_num; /* Physical PCB silkscreen number (1..7) */
} tcrt_channel_map_t;

/*
 * TESTING CONFIGURATION NOTE:
 * - When USE_TELEMETRY == 1 (Wi-Fi active for UDP logging/tuning):
 *   ADC2 cannot be used alongside Wi-Fi, disabling S7 (GPIO 25).
 *   We intentionally exclude S1 (GPIO 36) as well so the active 5-sensor
 *   window (S2..S6) remains symmetric around center sensor S4 (0 mm).
 * - When USE_TELEMETRY == 0 (Competition mode, Wi-Fi off):
 *   All 7 sensors (S1..S7) operate across the full -27 mm to +27 mm span.
 */

static const tcrt_channel_map_t s_sensor_map[TCRT5000_ACTIVE_CHANNELS] = {
#if USE_TELEMETRY
    { ADC_UNIT_1, ADC_CHANNEL_3, PIN_SENSOR_2, 2 }, // Index 0 -> S2: GPIO 39 (-18 mm)
    { ADC_UNIT_1, ADC_CHANNEL_6, PIN_SENSOR_3, 3 }, // Index 1 -> S3: GPIO 34 ( -9 mm)
    { ADC_UNIT_1, ADC_CHANNEL_7, PIN_SENSOR_4, 4 }, // Index 2 -> S4: GPIO 35 (  0 mm)
    { ADC_UNIT_1, ADC_CHANNEL_4, PIN_SENSOR_5, 5 }, // Index 3 -> S5: GPIO 32 ( +9 mm)
    { ADC_UNIT_1, ADC_CHANNEL_5, PIN_SENSOR_6, 6 }, // Index 4 -> S6: GPIO 33 (+18 mm)
#else
    { ADC_UNIT_1, ADC_CHANNEL_0, PIN_SENSOR_1, 1 }, // Index 0 -> S1: GPIO 36 (-27 mm)
    { ADC_UNIT_1, ADC_CHANNEL_3, PIN_SENSOR_2, 2 }, // Index 1 -> S2: GPIO 39 (-18 mm)
    { ADC_UNIT_1, ADC_CHANNEL_6, PIN_SENSOR_3, 3 }, // Index 2 -> S3: GPIO 34 ( -9 mm)
    { ADC_UNIT_1, ADC_CHANNEL_7, PIN_SENSOR_4, 4 }, // Index 3 -> S4: GPIO 35 (  0 mm)
    { ADC_UNIT_1, ADC_CHANNEL_4, PIN_SENSOR_5, 5 }, // Index 4 -> S5: GPIO 32 ( +9 mm)
    { ADC_UNIT_1, ADC_CHANNEL_5, PIN_SENSOR_6, 6 }, // Index 5 -> S6: GPIO 33 (+18 mm)
    { ADC_UNIT_2, ADC_CHANNEL_8, PIN_SENSOR_7, 7 }, // Index 6 -> S7: GPIO 25 (+27 mm)
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
    ESP_LOGW(TAG, "USE_TELEMETRY=1: Sensor 1&7 (GPIO %d & %d / ADC2) disabled to prevent Wi-Fi conflict",PIN_SENSOR_1, PIN_SENSOR_7);
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

esp_err_t tcrt5000_read_compensated(int net_values[LINE_SENSOR_MAX_COUNT],
                                    int amb_values[LINE_SENSOR_MAX_COUNT])
{
    if (net_values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int val_on[LINE_SENSOR_MAX_COUNT]  = {0};
    int val_off[LINE_SENSOR_MAX_COUNT] = {0};

    /* 1. Read with IR LEDs ON (Total = Reflected IR + Ambient) */
    tcrt5000_set_emitters(true);
    esp_rom_delay_us(80); // Wait for TCRT5000 phototransistor + 100pF cap to settle
    esp_err_t err1 = tcrt5000_read_raw(val_on);

    /* 2. Read with IR LEDs OFF (Ambient room/sunlight only) */
    tcrt5000_set_emitters(false);
    esp_rom_delay_us(80);
    esp_err_t err2 = tcrt5000_read_raw(val_off);

    if (err1 != ESP_OK || err2 != ESP_OK) {
        return (err1 != ESP_OK) ? err1 : err2;
    }

    /* Leave emitters OFF between samples to save battery (~50 mA saved!) */
    for (int i = 0; i < TCRT5000_ACTIVE_CHANNELS; i++) {
        if (i >= TCRT5000_ACTIVE_CHANNELS || val_on[i] < 0 || val_off[i] < 0) {
            net_values[i] = -1;
            if (amb_values != NULL) amb_values[i] = -1;
            continue;
        }

        /* Because phototransistor pulls A1..A7 LOW when illuminated:
         * Bright white (high IR) -> val_on is LOW, val_off is HIGH -> (val_off - val_on) is LARGE
         * Black line (low IR)    -> val_on ≈ val_off -> (val_off - val_on) is NEAR ZERO 
         * val_on = ambience + raw_data 
         * val_off = ambience*/

        int diff = val_off[i] - val_on[i];
        net_values[i] = (diff > 0) ? diff : 0;

        if (amb_values != NULL) {
            amb_values[i] = val_off[i];
        }   
    }
    return ESP_OK;
}
