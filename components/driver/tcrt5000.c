#include "tcrt5000.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "tcrt5000";

static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_oneshot_unit_handle_t s_adc2_handle = NULL;
static tcrt_profile_t            s_profile     = TCRT_PROFILE_5S;
static bool                      s_initialized = false;

typedef struct {
    adc_unit_t    unit;
    adc_channel_t channel;
    int           gpio_num;
    int           sensor_num; /* Physical PCB silkscreen number (1..7) */
} tcrt_channel_map_t;

/* Fixed physical mapping: index 0..6 == S1..S7 (S1 = Left -27mm .. S7 = Right +27mm) */
static const tcrt_channel_map_t s_phys_map[LINE_SENSOR_MAX_COUNT] = {
    { ADC_UNIT_1, ADC_CHANNEL_0, PIN_SENSOR_1, 1 }, /* [0] S1: GPIO 36 (-27 mm, Left) */
    { ADC_UNIT_1, ADC_CHANNEL_3, PIN_SENSOR_2, 2 }, /* [1] S2: GPIO 39 (-18 mm)       */
    { ADC_UNIT_1, ADC_CHANNEL_6, PIN_SENSOR_3, 3 }, /* [2] S3: GPIO 34 ( -9 mm)       */
    { ADC_UNIT_1, ADC_CHANNEL_7, PIN_SENSOR_4, 4 }, /* [3] S4: GPIO 35 (  0 mm, Mid)  */
    { ADC_UNIT_1, ADC_CHANNEL_4, PIN_SENSOR_5, 5 }, /* [4] S5: GPIO 32 ( +9 mm)       */
    { ADC_UNIT_1, ADC_CHANNEL_5, PIN_SENSOR_6, 6 }, /* [5] S6: GPIO 33 (+18 mm)       */
    { ADC_UNIT_2, ADC_CHANNEL_8, PIN_SENSOR_7, 7 }, /* [6] S7: GPIO 25 (+27 mm, ADC2) */
};

static inline bool is_slot_active(int slot, tcrt_profile_t profile)
{
    if (profile == TCRT_PROFILE_7S) {
        return (slot >= 0 && slot < LINE_SENSOR_MAX_COUNT);
    }
    /* TCRT_PROFILE_5S: symmetric center window S2..S6 (slots 1..5) */
    return (slot >= 1 && slot <= 5);
}

void tcrt5000_set_emitters(bool enable)
{
    gpio_set_level(PIN_SENSOR_CTRL, enable ? 1 : 0);
}

static esp_err_t configure_adc_units(tcrt_profile_t profile)
{
    esp_err_t ret;
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };

    /* 1. Ensure ADC1 unit exists and configure all ADC1 slots (S1..S6) */
    if (s_adc1_handle == NULL) {
        adc_oneshot_unit_init_cfg_t init_cfg1 = {
            .unit_id  = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ret = adc_oneshot_new_unit(&init_cfg1, &s_adc1_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC1 init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    for (int i = 0; i < 6; i++) {
        ret = adc_oneshot_config_channel(s_adc1_handle, s_phys_map[i].channel, &chan_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to config S%d (GPIO %d): %s",
                     s_phys_map[i].sensor_num, s_phys_map[i].gpio_num, esp_err_to_name(ret));
            return ret;
        }
    }

    /* 2. Manage ADC2 (S7 / GPIO25) strictly according to profile */
    if (profile == TCRT_PROFILE_7S) {
        if (s_adc2_handle == NULL) {
            adc_oneshot_unit_init_cfg_t init_cfg2 = {
                .unit_id  = ADC_UNIT_2,
                .ulp_mode = ADC_ULP_MODE_DISABLE,
            };
            ret = adc_oneshot_new_unit(&init_cfg2, &s_adc2_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "ADC2 init failed (is Wi-Fi OFF?): %s", esp_err_to_name(ret));
                return ret;
            }
        }
        ret = adc_oneshot_config_channel(s_adc2_handle, s_phys_map[6].channel, &chan_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to config S7 (GPIO %d / ADC2): %s",
                     s_phys_map[6].gpio_num, esp_err_to_name(ret));
            return ret;
        }
    } else {
        /* TCRT_PROFILE_5S: release ADC2 so Wi-Fi can start/run without conflict */
        if (s_adc2_handle != NULL) {
            adc_oneshot_del_unit(s_adc2_handle);
            s_adc2_handle = NULL;
        }
    }

    s_profile = profile;
    ESP_LOGI(TAG, "TCRT5000 active profile: %d sensors (%s)",
             (int)profile, (profile == TCRT_PROFILE_7S) ? "S1..S7, ADC1+ADC2" : "S2..S6, ADC1 only");
    return ESP_OK;
}

esp_err_t tcrt5000_init(tcrt_profile_t profile)
{
    if (s_initialized) {
        return tcrt5000_set_profile(profile);
    }

    /* Configure low-side N-MOSFET IR emitter control pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_SENSOR_CTRL),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CTRL GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    tcrt5000_set_emitters(false);

    ret = configure_adc_units(profile);
    if (ret != ESP_OK) {
        return ret;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t tcrt5000_set_profile(tcrt_profile_t profile)
{
    if (!s_initialized) {
        return tcrt5000_init(profile);
    }
    if (profile == s_profile &&
        ((profile == TCRT_PROFILE_5S && s_adc2_handle == NULL) ||
         (profile == TCRT_PROFILE_7S && s_adc2_handle != NULL))) {
        return ESP_OK;
    }
    return configure_adc_units(profile);
}

tcrt_profile_t tcrt5000_get_profile(void)
{
    return s_profile;
}

bool tcrt5000_is_initialized(void)
{
    return s_initialized;
}

esp_err_t tcrt5000_deinit(void)
{
    tcrt5000_set_emitters(false);

    if (s_adc1_handle != NULL) {
        adc_oneshot_del_unit(s_adc1_handle);
        s_adc1_handle = NULL;
    }
    if (s_adc2_handle != NULL) {
        adc_oneshot_del_unit(s_adc2_handle);
        s_adc2_handle = NULL;
    }
    s_initialized = false;
    return ESP_OK;
}

static esp_err_t read_active_avg(int out_values[LINE_SENSOR_MAX_COUNT], uint8_t oversample)
{
    if (oversample < 1) oversample = 1;
    if (oversample > 8) oversample = 8;

    int32_t acc[LINE_SENSOR_MAX_COUNT] = {0};
    int     cnt[LINE_SENSOR_MAX_COUNT] = {0};
    esp_err_t overall_ret = ESP_OK;

    for (uint8_t s = 0; s < oversample; s++) {
        for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
            if (!is_slot_active(i, s_profile)) {
                continue;
            }
            adc_oneshot_unit_handle_t handle =
                (s_phys_map[i].unit == ADC_UNIT_2) ? s_adc2_handle : s_adc1_handle;
            if (handle == NULL) {
                overall_ret = ESP_ERR_INVALID_STATE;
                continue;
            }
            int sample = 0;
            esp_err_t ret = adc_oneshot_read(handle, s_phys_map[i].channel, &sample);
            if (ret == ESP_OK) {
                acc[i] += sample;
                cnt[i]++;
            } else {
                overall_ret = ret;
            }
        }
    }

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        if (!is_slot_active(i, s_profile) || cnt[i] == 0) {
            out_values[i] = -1;
        } else {
            out_values[i] = (int)((acc[i] + (cnt[i] / 2)) / cnt[i]);
        }
    }
    return overall_ret;
}

esp_err_t tcrt5000_read_raw(int raw_values[LINE_SENSOR_MAX_COUNT])
{
    if (raw_values == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized)     return ESP_ERR_INVALID_STATE;
    return read_active_avg(raw_values, 1);
}

esp_err_t tcrt5000_read_compensated_avg(int net_values[LINE_SENSOR_MAX_COUNT],
                                        int amb_values[LINE_SENSOR_MAX_COUNT],
                                        uint8_t oversample)
{
    if (net_values == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized)     return ESP_ERR_INVALID_STATE;

    int val_on[LINE_SENSOR_MAX_COUNT];
    int val_off[LINE_SENSOR_MAX_COUNT];

    /* 1. Read with IR LEDs ON (Reflected IR + Ambient) */
    tcrt5000_set_emitters(true);
    esp_rom_delay_us(80);
    esp_err_t err1 = read_active_avg(val_on, oversample);

    /* 2. Read with IR LEDs OFF (Ambient only; leave OFF afterward to save power) */
    tcrt5000_set_emitters(false);
    esp_rom_delay_us(80);
    esp_err_t err2 = read_active_avg(val_off, oversample);

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        if (!is_slot_active(i, s_profile) || val_on[i] < 0 || val_off[i] < 0) {
            net_values[i] = -1;
            if (amb_values != NULL) amb_values[i] = -1;
            continue;
        }
        /* Phototransistor pulls ADC LOW when illuminated:
         *   White floor -> high IR reflection -> val_on LOW  -> (val_off - val_on) LARGE
         *   Black line  -> low IR reflection  -> val_on ~ off -> (val_off - val_on) NEAR 0 */
        int diff = val_off[i] - val_on[i];
        net_values[i] = (diff > 0) ? diff : 0;
        if (amb_values != NULL) {
            amb_values[i] = val_off[i];
        }
    }

    if (err1 != ESP_OK) return err1;
    return err2;
}

esp_err_t tcrt5000_read_compensated(int net_values[LINE_SENSOR_MAX_COUNT],
                                    int amb_values[LINE_SENSOR_MAX_COUNT])
{
    return tcrt5000_read_compensated_avg(net_values, amb_values, TCRT5000_OVERSAMPLE_COUNT);
}
