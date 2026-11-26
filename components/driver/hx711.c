#include <stddef.h>
#include "hx711.h"
#include "config.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static gpio_num_t   s_dout = GPIO_NUM_NC;
static gpio_num_t   s_sck  = GPIO_NUM_NC;
static int          s_extra_pulses = 1; /* 1 = Gain 128 (25 total), 3 = Gain 64 (27 total) */
static portMUX_TYPE s_mux          = portMUX_INITIALIZER_UNLOCKED;

esp_err_t hx711_init(gpio_num_t dout_pin, gpio_num_t sck_pin, int gain)
{
    if (gain == 128) {
        s_extra_pulses = 1;
    } else if (gain == 64) {
        s_extra_pulses = 3;
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    s_dout = dout_pin;
    s_sck  = sck_pin;

    gpio_config_t in_cfg = {
        .pin_bit_mask = 1ULL << s_dout,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&in_cfg);
    if (err != ESP_OK) return err;

    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << s_sck,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&out_cfg);
    if (err != ESP_OK) return err;

    /* Ensure clock starts LOW (active mode) */
    gpio_set_level(s_sck, 0);
    return ESP_OK;
}

void hx711_deinit(void)
{
    if (s_sck != GPIO_NUM_NC) {
        /* Holding SCK HIGH > 60 us puts the HX711 into power-down mode (< 1 uA) */
        gpio_set_level(s_sck, 1);
        esp_rom_delay_us(80);
        gpio_reset_pin(s_sck);
        s_sck = GPIO_NUM_NC;
    }
    if (s_dout != GPIO_NUM_NC) {
        gpio_reset_pin(s_dout);
        s_dout = GPIO_NUM_NC;
    }
}

bool hx711_is_ready(void)
{
    if (s_dout == GPIO_NUM_NC) {
        return false;
    }
    return gpio_get_level(s_dout) == 0;
}


esp_err_t hx711_read_raw(int32_t *value, uint32_t timeout_ms)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dout == GPIO_NUM_NC || s_sck == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_STATE;
    }

    int64_t deadline = esp_timer_get_time() + ((int64_t)timeout_ms * 1000);
    while (!hx711_is_ready()) {
        if (esp_timer_get_time() > deadline) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    uint32_t raw = 0;

    /* Critical section: keep each SCK HIGH pulse < 50 us so HX711 doesn't sleep */
    portENTER_CRITICAL(&s_mux);

    for (int i = 0; i < 24; i++) {
        gpio_set_level(s_sck, 1);
        esp_rom_delay_us(1);
        raw = (raw << 1) | (uint32_t)gpio_get_level(s_dout);
        gpio_set_level(s_sck, 0);
        esp_rom_delay_us(1);
    }

    /* Extra pulses to set channel & gain for the NEXT sample */
    for (int i = 0; i < s_extra_pulses; i++) {
        gpio_set_level(s_sck, 1);
        esp_rom_delay_us(1);
        gpio_set_level(s_sck, 0);
        esp_rom_delay_us(1);
    }

    portEXIT_CRITICAL(&s_mux);

    /* Sign-extend 24-bit two's complement to 32-bit signed int */
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }

    *value = (int32_t)raw;
    return ESP_OK;
}
