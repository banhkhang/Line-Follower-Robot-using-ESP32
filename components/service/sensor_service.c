#include "sensor_service.h"
#include "tcrt5000.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_service";

/* Physical spacing between adjacent TCRT5000 sensors in mm (adjust to your PCB) */
#define SENSOR_PITCH_MM         9.0f
#define NORM_NOISE_DEADZONE     5.0f   /* Ignore <5% floor variation */
#define LINE_DETECT_SUM_MIN     25.0f   /* Minimum sum(S_i) to consider line valid */
#define SAMPLING_PERIOD_MS           10      /*(ms) per 1s: 1000ms/10ms = 100 Hz/*/
static bool s_enabled = false;
static int  s_r_white[LINE_SENSOR_MAX_COUNT];
static int  s_r_black[LINE_SENSOR_MAX_COUNT];
static float s_last_position_mm = 0.0f;

/* Symmetric sensor coordinates x_i in mm */
#if USE_TELEMETRY
/* 6 active sensors: center is S4 */
static const float s_pos_mm[TCRT5000_ACTIVE_CHANNELS] = {
    -3.0f * SENSOR_PITCH_MM,
    -2.0f * SENSOR_PITCH_MM,
    -1.0f * SENSOR_PITCH_MM,
     0.0f,
    +1.0f * SENSOR_PITCH_MM,
    +2.0f * SENSOR_PITCH_MM,
};
#else
/* 7 active sensors: center is S4 */
static const float s_pos_mm[TCRT5000_ACTIVE_CHANNELS] = {
    -3.0f * SENSOR_PITCH_MM,
    -2.0f * SENSOR_PITCH_MM,
    -1.0f * SENSOR_PITCH_MM,
     0.0f,
    +1.0f * SENSOR_PITCH_MM,
    +2.0f * SENSOR_PITCH_MM,
    +3.0f * SENSOR_PITCH_MM
};
#endif

static void wait_for_boot_button_click(void)
{
    /* 1. Ensure button is released first (prevents bleed-over from previous stage) */
    while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 2. Wait for button press */
    while (gpio_get_level(PIN_BOOT_BUTTON) != 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Debounce press
}

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

    /* Safe fallback defaults before calibration runs */
    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        s_r_white[i] = 1000;
        s_r_black[i] = 3200;
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

esp_err_t sensor_service_calibrate_sweep(void)
{
    if (!s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Ensure BOOT button GPIO is configured as input with pull-up */
    gpio_reset_pin(PIN_BOOT_BUTTON);
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << PIN_BOOT_BUTTON,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn);

    /* STAGE 1: Wait for BOOT button to start sweeping */
    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "CALIBRATION READY: Press BOOT button to START sweep");
    ESP_LOGI(TAG, "==================================================");
    wait_for_boot_button_click();

    while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* STAGE 2: Sweep until BOOT button is pressed again */
    int min_val[LINE_SENSOR_MAX_COUNT];
    int max_val[LINE_SENSOR_MAX_COUNT];
    int sample[LINE_SENSOR_MAX_COUNT];

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        min_val[i] = 4095;
        max_val[i] = 0;
    }

    ESP_LOGI(TAG, "SWEEPING... Wave sensors across white & black line. Press BOOT to FINISH.");

    uint32_t blink_ms = 0;
    while (gpio_get_level(PIN_BOOT_BUTTON) != 0) {
        if (tcrt5000_read_raw(sample) == ESP_OK) {
            for (int i = 0; i < TCRT5000_ACTIVE_CHANNELS; i++) {
                if (sample[i] >= 0) {
                    if (sample[i] < min_val[i]) min_val[i] = sample[i];
                    if (sample[i] > max_val[i]) max_val[i] = sample[i];
                }
            }
        }

        /* Fast 100 ms blink on USER_LED to indicate active sweeping */
        blink_ms += SAMPLING_PERIOD_MS;
        gpio_set_level(PIN_USER_LED, (blink_ms / 100) % 2);

        vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD_MS));
    }
        /* Wait for button release after stopping sweep */
    while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    gpio_set_level(PIN_USER_LED, 0);

    /* STAGE 3: Commit calibration values with a 10% guard margin */
    ESP_LOGI(TAG, "CALIBRATION COMPLETE:");
    for (int i = 0; i < TCRT5000_ACTIVE_CHANNELS; i++) {
        int span = max_val[i] - min_val[i];
        if (span < 200) {
            ESP_LOGW(TAG, "  S%d low contrast (min=%d, max=%d) - check sweep!",
                     i + 1, min_val[i], max_val[i]);
            s_r_white[i] = min_val[i];
            s_r_black[i] = max_val[i];
        } else {
            /* 10% inward margin absorbs minor floor/tape sheen variations */
            int margin = span / 10;
            s_r_white[i] = min_val[i] + margin;
            s_r_black[i] = max_val[i] - margin;
        }
        ESP_LOGI(TAG, "  S%d -> R_white=%4d, R_black=%4d (span=%4d)",
                 i + 1, s_r_white[i], s_r_black[i], s_r_black[i] - s_r_white[i]);
    }

    return ESP_OK;
}

esp_err_t sensor_service_read(line_sensor_reading_t *out_reading)
{
    if (!s_enabled) return ESP_ERR_INVALID_STATE;
    if (out_reading == NULL) return ESP_ERR_INVALID_ARG;
    
    esp_err_t err = tcrt5000_read_raw(out_reading->raw);
    if (err != ESP_OK) {
        return err;
    }
    
    /* WEIGHTED AVERAGE METHOD IMPLEMENTATION*/
    out_reading->active_count = TCRT5000_ACTIVE_CHANNELS;
    float weighted_sum = 0.0f;
    float total_weight = 0.0f;

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        if (i >= TCRT5000_ACTIVE_CHANNELS || out_reading->raw[i] < 0) {
            out_reading->norm[i] = 0.0f;
            continue;
        }

        int denom = s_r_black[i] - s_r_white[i];
        float s_i = 0.0f;

        if (denom > 0) {
            s_i = ((float)(out_reading->raw[i] - s_r_white[i]) / (float)denom) * 100.0f;
        }

        /* Clamp to [0.0, 100.0] */
        if (s_i < 0.0f) s_i = 0.0f;
        if (s_i > 100.0f) s_i = 100.0f;

        /* Apply floor noise deadzone */
        if (s_i < NORM_NOISE_DEADZONE) {
            s_i = 0.0f;
        }

        out_reading->norm[i] = s_i;
        weighted_sum += s_i * s_pos_mm[i];
        total_weight += s_i;
    }

    if (total_weight >= LINE_DETECT_SUM_MIN) {
        out_reading->line_detected = true;
        out_reading->position_mm = weighted_sum / total_weight;
        s_last_position_mm = out_reading->position_mm;
    } else {
        /* Line lost: hold last known side so PID knows which way to turn back */
        out_reading->line_detected = false;
        out_reading->position_mm = s_last_position_mm;
    }

    return ESP_OK;
}
