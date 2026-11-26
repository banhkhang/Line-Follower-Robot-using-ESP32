#include "usr_led.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    LED_MODE_STATIC = 0,
    LED_MODE_SQUARE_BLINK,
    LED_MODE_RUN_RHYTHM,
} led_mode_t;

/* 100 ms per step -> 40 steps = 4.0 s repeating cycle: 1 - 2 - 1 - 1 - 2 */
static const uint8_t s_run_pattern[] = {
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0,   /* 2: 0.1s ON + 0.9s OFF = 1.0s */
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0,   /* 2: 0.1s ON + 0.9s OFF = 1.0s */
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0,   /* 1-1: 0.1s ON + 0.4s OFF + 0.1s ON + 0.4s OFF = 1.0s */
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0    /* 2: 0.1s ON + 0.9s OFF = 1.0s */
};
#define RUN_PATTERN_LEN (sizeof(s_run_pattern) / sizeof(s_run_pattern[0]))

static bool               s_initialized = false;
static bool               s_led_state   = false;
static led_mode_t         s_mode        = LED_MODE_STATIC;
static uint32_t           s_step_idx    = 0;
static esp_timer_handle_t s_led_timer   = NULL;

static void stop_timer_if_running(void)
{
    if (s_led_timer != NULL && esp_timer_is_active(s_led_timer)) {
        esp_timer_stop(s_led_timer);
    }
}

static void led_timer_cb(void *arg)
{
    if (s_mode == LED_MODE_SQUARE_BLINK) {
        s_led_state = !s_led_state;
        gpio_set_level(PIN_USER_LED, s_led_state ? 1 : 0);
    } else if (s_mode == LED_MODE_RUN_RHYTHM) {
        uint8_t level = s_run_pattern[s_step_idx];
        s_step_idx = (s_step_idx + 1) % RUN_PATTERN_LEN;
        s_led_state = (level != 0);
        gpio_set_level(PIN_USER_LED, level);
    }
}

esp_err_t usr_led_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    gpio_reset_pin(PIN_USER_LED);
    gpio_config_t led = {
        .pin_bit_mask = 1ULL << PIN_USER_LED,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&led);
    if (err != ESP_OK) {
        return err;
    }

    s_led_state = false;
    s_mode      = LED_MODE_STATIC;
    gpio_set_level(PIN_USER_LED, 0);

    const esp_timer_create_args_t timer_args = {
        .callback = &led_timer_cb,
        .name     = "usr_led_timer",
    };
    err = esp_timer_create(&timer_args, &s_led_timer);
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    return ESP_OK;
}

void usr_led_set(bool on)
{
    if (!s_initialized) return;

    stop_timer_if_running();
    s_mode      = LED_MODE_STATIC;
    s_led_state = on;
    gpio_set_level(PIN_USER_LED, on ? 1 : 0);
}

void usr_led_blink_poll(uint32_t half_period_ms)
{
    if (!s_initialized) return;

    if (half_period_ms == 0) {
        usr_led_set(false);
        return;
    }

    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool state = ((now_ms / half_period_ms) % 2) == 0;
    if (state != s_led_state) {
        s_led_state = state;
        gpio_set_level(PIN_USER_LED, state ? 1 : 0);
    }
}

void usr_led_start_blink(uint32_t half_period_ms)
{
    if (!s_initialized) return;

    stop_timer_if_running();
    if (half_period_ms == 0) {
        usr_led_set(false);
        return;
    }

    s_mode      = LED_MODE_SQUARE_BLINK;
    s_led_state = true;
    gpio_set_level(PIN_USER_LED, 1);
    esp_timer_start_periodic(s_led_timer, (uint64_t)half_period_ms * 1000ULL);
}

void usr_led_start_run_pattern(void)
{
    if (!s_initialized) return;

    stop_timer_if_running();
    s_mode      = LED_MODE_RUN_RHYTHM;
    s_step_idx  = 1; /* Step 0 applied immediately below */
    s_led_state = true;
    gpio_set_level(PIN_USER_LED, s_run_pattern[0]);
    esp_timer_start_periodic(s_led_timer, 100ULL * 1000ULL); /* 100 ms base tick */
}