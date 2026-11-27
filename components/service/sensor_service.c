#include "config.h"
#include "robot_params.h"
#include "sensor_service.h"
#include "tcrt5000.h"
#include "usr_led.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "sensor_service";

#define SENSOR_PITCH_MM         9.0f
#define NORM_NOISE_DEADZONE     5.0f    /* Ignore <5% floor variation for centroid */
#define LINE_DETECT_SUM_MIN     25.0f   /* Minimum sum(norm_i) fallback threshold */
#define CALIB_SAMPLE_PERIOD_MS  10      /* 100 Hz during sweep calibration */

#define NVS_CAL_NAMESPACE       "line_cal"
#define NVS_CAL_KEY             "tcrt7_v1"
#define NVS_CAL_MAGIC           0x4C433756U /* 'LC7V' */

typedef struct {
    uint32_t magic;
    int32_t  r_white[LINE_SENSOR_MAX_COUNT];
    int32_t  r_black[LINE_SENSOR_MAX_COUNT];
} sensor_calib_blob_t;

static bool  s_enabled          = false;
static bool  s_has_valid_calib  = false;
static int   s_r_white[LINE_SENSOR_MAX_COUNT];
static int   s_r_black[LINE_SENSOR_MAX_COUNT];
static float s_last_position_mm = 0.0f;

static int      s_amb_min[LINE_SENSOR_MAX_COUNT];
static int      s_amb_max[LINE_SENSOR_MAX_COUNT];
static uint64_t s_amb_sum[LINE_SENSOR_MAX_COUNT];
static uint32_t s_amb_samples = 0;

/*
 * Physical PCB coordinates x_i [mm] for slots 0..6 (S1..S7):
 *   S1 is on the LEFT  (-27 mm), S4 is CENTER (0 mm), S7 is on the RIGHT (+27 mm).
 * Note: In the outer-loop SMC ISO vehicle frame (+Y = Left), e2_smc_m = -position_mm * 1e-3.
 */
static const float s_phys_pos_mm[LINE_SENSOR_MAX_COUNT] = {
    -3.0f * SENSOR_PITCH_MM,   /* [0] S1: -27.0 mm (Left)  */
    -2.0f * SENSOR_PITCH_MM,   /* [1] S2: -18.0 mm         */
    -1.0f * SENSOR_PITCH_MM,   /* [2] S3:  -9.0 mm         */
     0.0f,                     /* [3] S4:   0.0 mm (Center)*/
    +1.0f * SENSOR_PITCH_MM,   /* [4] S5:  +9.0 mm         */
    +2.0f * SENSOR_PITCH_MM,   /* [5] S6: +18.0 mm         */
    +3.0f * SENSOR_PITCH_MM,   /* [6] S7: +27.0 mm (Right) */
};

static void reset_amb_stats(void)
{
    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        s_amb_min[i] = 4095;
        s_amb_max[i] = 0;
        s_amb_sum[i] = 0;
    }
    s_amb_samples = 0;
}

esp_err_t sensor_service_load_calib_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_CAL_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No saved 7-sensor calibration in NVS (%s) - using defaults",
                 esp_err_to_name(err));
        s_has_valid_calib = false;
        return err;
    }

    sensor_calib_blob_t blob;
    size_t sz = sizeof(blob);
    err = nvs_get_blob(h, NVS_CAL_KEY, &blob, &sz);
    nvs_close(h);

    if (err != ESP_OK || sz != sizeof(blob) || blob.magic != NVS_CAL_MAGIC) {
        ESP_LOGW(TAG, "Invalid NVS calibration blob - using defaults");
        s_has_valid_calib = false;
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        s_r_white[i] = (int)blob.r_white[i];
        s_r_black[i] = (int)blob.r_black[i];
    }
    s_has_valid_calib = true;

    ESP_LOGI(TAG, "Loaded 7-sensor calibration from NVS:");
    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        ESP_LOGI(TAG, "  S%d -> R_white=%4d, R_black=%4d (span=%4d)",
                 i + 1, s_r_white[i], s_r_black[i], s_r_white[i] - s_r_black[i]);
    }
    return ESP_OK;
}

esp_err_t sensor_service_save_calib_nvs(void)
{
    sensor_calib_blob_t blob;
    blob.magic = NVS_CAL_MAGIC;
    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        blob.r_white[i] = (int32_t)s_r_white[i];
        blob.r_black[i] = (int32_t)s_r_black[i];
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_CAL_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for calibration save: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(h, NVS_CAL_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err == ESP_OK) {
        s_has_valid_calib = true;
        ESP_LOGI(TAG, "Saved 7-sensor calibration to NVS successfully");
    } else {
        ESP_LOGE(TAG, "Failed to save NVS calibration: %s", esp_err_to_name(err));
    }
    return err;
}

bool sensor_service_has_valid_calib(void)
{
    return s_has_valid_calib;
}

esp_err_t sensor_service_init(tcrt_profile_t initial_profile)
{
    /* Safe fallback defaults for ambient-compensated readings (white > black) */
    if (!s_has_valid_calib) {
        for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
            s_r_white[i] = 2600;
            s_r_black[i] = 300;
        }
        (void)sensor_service_load_calib_nvs();
    }

    esp_err_t err = tcrt5000_init(initial_profile);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCRT5000 init failed (%s)", esp_err_to_name(err));
        s_enabled = false;
        return err;
    }

    reset_amb_stats();
    s_enabled = true;
    return ESP_OK;
}

void sensor_service_deinit(void)
{
    if (!s_enabled) return;
    tcrt5000_deinit();
    s_enabled = false;
}

bool sensor_service_is_enabled(void)
{
    return s_enabled;
}

esp_err_t sensor_service_set_profile(tcrt_profile_t profile)
{
    if (!s_enabled) {
        return sensor_service_init(profile);
    }
    esp_err_t err = tcrt5000_set_profile(profile);
    if (err == ESP_OK) {
        reset_amb_stats();
    }
    return err;
}

tcrt_profile_t sensor_service_get_profile(void)
{
    return tcrt5000_get_profile();
}

esp_err_t sensor_service_calibrate_sweep(void)
{
    /* Switch hardware to full 7-sensor profile (caller must have turned Wi-Fi OFF!) */
    esp_err_t err = sensor_service_set_profile(TCRT_PROFILE_7S);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot enter 7-sensor calibration: %s", esp_err_to_name(err));
        return err;
    }

    /* Ensure BOOT button is released before starting sweep */
    while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    int min_val[LINE_SENSOR_MAX_COUNT];
    int max_val[LINE_SENSOR_MAX_COUNT];
    int sample[LINE_SENSOR_MAX_COUNT];
    int amb_sample[LINE_SENSOR_MAX_COUNT];

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        min_val[i] = 4095;
        max_val[i] = 0;
    }

    /* LED1 SOLID ON throughout calibration as requested */
    usr_led_set(true);
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, "7-SENSOR CALIBRATION ACTIVE (LED1 ON): Wave bar across");
    ESP_LOGI(TAG, "white floor & black line. Press BOOT to FINISH and SAVE.");
    ESP_LOGI(TAG, "============================================================");

    while (gpio_get_level(PIN_BOOT_BUTTON) != 0) {
        if (tcrt5000_read_compensated_avg(sample, amb_sample, 2) == ESP_OK) {
            for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
                if (sample[i] >= 0) {
                    if (sample[i] < min_val[i]) min_val[i] = sample[i];
                    if (sample[i] > max_val[i]) max_val[i] = sample[i];
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(CALIB_SAMPLE_PERIOD_MS));
    }

    /* Debounce & wait for BOOT release */
    vTaskDelay(pdMS_TO_TICKS(50));
    while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Commit ambient-compensated calibration values (white = high diff, black = low diff) */
    ESP_LOGI(TAG, "CALIBRATION COMPLETE (7 sensors, ambient-compensated):");
    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        int span = max_val[i] - min_val[i];
        if (span < 200) {
            ESP_LOGW(TAG, "  S%d low contrast (black=%d, white=%d) - check sweep!",
                     i + 1, min_val[i], max_val[i]);
            s_r_black[i] = min_val[i];
            s_r_white[i] = max_val[i];
        } else {
            int margin   = span / 10; /* 10% inward guard margin */
            s_r_black[i] = min_val[i] + margin;
            s_r_white[i] = max_val[i] - margin;
        }
        ESP_LOGI(TAG, "  S%d -> R_white=%4d, R_black=%4d (span=%4d)",
                 i + 1, s_r_white[i], s_r_black[i], s_r_white[i] - s_r_black[i]);
    }

    err = sensor_service_save_calib_nvs();
    reset_amb_stats();

    /* Turn LED1 OFF once calibration completes */
    usr_led_set(false);
    return err;
}

esp_err_t sensor_service_read(line_sensor_reading_t *out_reading)
{
    if (!s_enabled)           return ESP_ERR_INVALID_STATE;
    if (out_reading == NULL)  return ESP_ERR_INVALID_ARG;

    int amb_raw[LINE_SENSOR_MAX_COUNT];
    esp_err_t err = tcrt5000_read_compensated(out_reading->raw, amb_raw);
    if (err != ESP_OK) {
        return err;
    }

    tcrt_profile_t prof = tcrt5000_get_profile();
    out_reading->profile = prof;

    s_amb_samples++;
    float   weighted_sum = 0.0f;
    float   total_weight = 0.0f;
    float   total_i      = 0.0f;
    uint8_t n_active     = 0;

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        if (out_reading->raw[i] < 0) {
            out_reading->norm[i]        = 0.0f;
            out_reading->i_eff[i]       = 0.0f;
            out_reading->amb.current[i] = -1;
            out_reading->amb.min[i]     = -1;
            out_reading->amb.max[i]     = -1;
            out_reading->amb.avg[i]     = 0.0f;
            continue;
        }

        /* Ambient statistics */
        int a = amb_raw[i];
        if (a < s_amb_min[i]) s_amb_min[i] = a;
        if (a > s_amb_max[i]) s_amb_max[i] = a;
        s_amb_sum[i] += (uint64_t)a;

        out_reading->amb.current[i] = a;
        out_reading->amb.min[i]     = s_amb_min[i];
        out_reading->amb.max[i]     = s_amb_max[i];
        out_reading->amb.avg[i]     = (float)s_amb_sum[i] / (float)s_amb_samples;

        /* Normalize: R_white -> 0.0% (white), R_black -> 100.0% (black) */
        int denom = s_r_white[i] - s_r_black[i];
        float s_pct = 0.0f;
        if (denom > 0) {
            s_pct = ((float)(s_r_white[i] - out_reading->raw[i]) / (float)denom) * 100.0f;
        }
        if (s_pct < 0.0f)   s_pct = 0.0f;
        if (s_pct > 100.0f) s_pct = 100.0f;

        /* SMC effective intensity matching SMC_controller14.m lines 601-602:
         *   I_eff = max(0, I_norm - 0.12); if (I_eff < 0.06) I_eff = 0; */
        float i_eff = (s_pct * 0.01f) - OUTER_GN_NOISE_OFFSET;
        if (i_eff < OUTER_GN_MIN_ACTIVE) {
            i_eff = 0.0f;
        }
        out_reading->i_eff[i] = i_eff;
        total_i += i_eff;
        if (i_eff > 0.15f) {
            n_active++;
        }

        /* Apply floor noise deadzone for telemetry display */
        if (s_pct < NORM_NOISE_DEADZONE) {
            s_pct = 0.0f;
        }
        out_reading->norm[i] = s_pct;
    }

    out_reading->total_i  = total_i;
    out_reading->n_active = n_active;

    /*
     * Tinh trong tam line (e2) bang I_eff (da tru nen quang hoc 12% & cat < 18% nhu
     * SMC_controller14.m) KET HOP cua so không gian quanh mat cuc dai (|y_k - y_peak| <= 20 mm):
     *  - Loai bo hoan toan nhieu nen trang o 2 mat ngoai cung S1/S7 (+-27 mm) trong He B (7S)
     *    va che do doc lap 7S (khong bi canh tay don 27 mm keo lech trong tam khi line o giua).
     *  - Khi line thuc su lech ra mep S1/S2 hoac S6/S7, mat cuc dai dich ra mep -> S1/S7 van
     *    tham gia binh thuong de giu tam nhin rong 7 mat!
     */
    int   peak_idx = -1;
    float peak_val = 0.0f;
    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        if (out_reading->i_eff[i] > peak_val) {
            peak_val = out_reading->i_eff[i];
            peak_idx = i;
        }
    }

    if (peak_idx >= 0) {
        float y_peak = s_phys_pos_mm[peak_idx];
        for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
            float w = out_reading->i_eff[i];
            if (w <= 0.0f) continue;
            float dy = s_phys_pos_mm[i] - y_peak;
            if (dy < 0.0f) dy = -dy;
            if (dy <= 20.0f) {
                weighted_sum += w * s_phys_pos_mm[i];
                total_weight += w;
            }
        }
    }

    if (total_i >= OUTER_LINE_TOTAL_I_MIN && total_weight >= OUTER_LINE_TOTAL_I_MIN) {
        out_reading->line_detected = true;

        float pos = weighted_sum / total_weight;
        if (prof == TCRT_PROFILE_5S) {
            /* Compensate 5-sensor edge compression beyond divergence threshold */
            float abs_pos = (pos >= 0.0f) ? pos : -pos;
            if (abs_pos > EDGE_CORR_START_MM) {
                float corr = EDGE_CORR_START_MM + (abs_pos - EDGE_CORR_START_MM) * EDGE_CORR_GAIN;
                pos = (pos >= 0.0f) ? corr : -corr;
            }
        }
        out_reading->position_mm = pos;
        s_last_position_mm       = pos;
    } else {
        out_reading->line_detected = false;
        out_reading->position_mm   = s_last_position_mm;
    }

    /*
     * Convert PCB coordinate convention (S1 Left = -27 mm) to SMC ISO vehicle frame
     * (Left = +Y, Right = -Y, CCW = +w):
     *   Line on LEFT -> position_mm < 0 -> e2_smc_m > 0 -> SMC commands w_cmd > 0 (turn LEFT).
     */
    out_reading->e2_smc_m = -out_reading->position_mm * 0.001f;
    return ESP_OK;
}
