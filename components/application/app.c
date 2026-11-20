/* app.c */

#include "app.h"
#include "ota.h"
#include "wifi_ap.h"
#include "telemetry.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STATUS_LED_GPIO   GPIO_NUM_23     /* JP4 Pin 1 */

/* JP1 (Motor A) */
#define MOTOR_A1_GPIO     PIN_MOTOR_A1    /* IO12 -> PWMA1 -> Q1 -> AIN1 */
#define MOTOR_A2_GPIO     PIN_MOTOR_A2    /* IO13 -> PWMA2 -> Q2 -> AIN2 */

/* JP2 (Motor B) */
#define MOTOR_B1_GPIO     GPIO_NUM_16     /* IO16 -> PWMB1 -> Q3 -> BIN1 */
#define MOTOR_B2_GPIO     GPIO_NUM_15     /* IO15 -> PWMB2 -> Q4 -> BIN2 */

/* PWM settings for DRV8871 + 6N137 optocouplers */
#define PWM_FREQ_HZ       10000           /* 10 kHz (well within 6N137 speed) */
#define PWM_RES           LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY      1023
#define TEST_SPEED_DUTY   520             /* ~50% speed for safe bench testing */

static const char *TAG = "app";

static void motors_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = PWM_RES,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    /* Configure PWMA1 (IO12) - default HIGH = AIN1 LOW */
    ledc_channel_config_t ch0_cfg = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_A1_GPIO,
        .duty           = PWM_MAX_DUTY,
        .hpoint         = 0
    };
    ledc_channel_config(&ch0_cfg);

    /* Configure PWMA2 (IO13) - default HIGH = AIN2 LOW */
    ledc_channel_config_t ch1_cfg = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_A2_GPIO,
        .duty           = PWM_MAX_DUTY,
        .hpoint         = 0
    };
    ledc_channel_config(&ch1_cfg);

    /* Configure PWMB1 (IO16) - default HIGH = BIN1 LOW */
    ledc_channel_config_t ch2_cfg = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_B1_GPIO,
        .duty           = PWM_MAX_DUTY,
        .hpoint         = 0
    };
    ledc_channel_config(&ch2_cfg);

    /* Configure PWMB2 (IO15) - default HIGH = BIN2 LOW */
    ledc_channel_config_t ch3_cfg = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_3,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_B2_GPIO,
        .duty           = PWM_MAX_DUTY,
        .hpoint         = 0
    };
    ledc_channel_config(&ch3_cfg);
}

/* --- JP1 (Motor A) Control --- */
static void motor_a_stop(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, PWM_MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void motor_a_forward(uint32_t speed_duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void motor_a_reverse(uint32_t speed_duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

/* --- JP2 (Motor B) Control --- */
static void motor_b_stop(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, PWM_MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, PWM_MAX_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

static void motor_b_forward(uint32_t speed_duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, speed_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

static void motor_b_reverse(uint32_t speed_duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, speed_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

static void motor_test_task(void *pvParameters)
{
    gpio_reset_pin(STATUS_LED_GPIO);
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        /* 1. Forward for 2 seconds (LED ON) */
        ESP_LOGI(TAG, "JP1 & JP2 Motors: FORWARD (~50%%)");
        gpio_set_level(STATUS_LED_GPIO, 1);
        motor_a_forward(TEST_SPEED_DUTY);
        motor_b_forward(TEST_SPEED_DUTY);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 2. Stop for 2 seconds (LED OFF) */
        ESP_LOGI(TAG, "JP1 & JP2 Motors: STOP");
        gpio_set_level(STATUS_LED_GPIO, 0);
        motor_a_stop();
        motor_b_stop();
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 3. Reverse for 2 seconds (LED ON) */
        ESP_LOGI(TAG, "JP1 & JP2 Motors: REVERSE (~50%%)");
        gpio_set_level(STATUS_LED_GPIO, 1);
        motor_a_reverse(TEST_SPEED_DUTY);
        motor_b_reverse(TEST_SPEED_DUTY);
        vTaskDelay(pdMS_TO_TICKS(2000));

        /* 4. Stop for 2 seconds (LED OFF) */
        ESP_LOGI(TAG, "JP1 & JP2 Motors: STOP");
        gpio_set_level(STATUS_LED_GPIO, 0);
        motor_a_stop();
        motor_b_stop();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_start(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    ESP_LOGW("BOOT", "================================");
    ESP_LOGW("BOOT", "ESP32 RESTART DETECTED");
    ESP_LOGW("BOOT", "Reset reason: %d", reason);
    ESP_LOGW("BOOT", "================================");

    /* 0. Initialize Motor A & B PWM immediately so all pins are held HIGH (motors OFF) */
    motors_init();
    motor_a_stop();
    motor_b_stop();

    /* 1. Start log capture and Wi-Fi stack (Required for OTA) */
    wifi_ap_init_base();
    telemetry_init();

    ESP_LOGI(TAG, "Starting up");

    /* 2. Open OTA window */
    ota_boot_window();

    /* 3. Start JP1 & JP2 motor test loop */
    ESP_LOGI(TAG, "Starting JP1 & JP2 motor test task");
    xTaskCreate(motor_test_task, "motor_test_task", 2048, NULL, 5, NULL);
}