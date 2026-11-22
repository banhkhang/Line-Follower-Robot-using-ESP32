#include "hx711.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE s_hx711_mux = portMUX_INITIALIZER_UNLOCKED;

static inline int hx711_clock_bit(const hx711_t *dev)
{
    int bit;
    /* Keep SCK HIGH strictly < 50 us so HX711 never enters power-down */
    portENTER_CRITICAL(&s_hx711_mux);
    gpio_set_level(dev->sck, 1);
    esp_rom_delay_us(1);
    bit = gpio_get_level(dev->dout);
    gpio_set_level(dev->sck, 0);
    portEXIT_CRITICAL(&s_hx711_mux);

    esp_rom_delay_us(1);
    return bit;
}

static inline void hx711_clock_pulse(const hx711_t *dev)
{
    portENTER_CRITICAL(&s_hx711_mux);
    gpio_set_level(dev->sck, 1);
    esp_rom_delay_us(1);
    gpio_set_level(dev->sck, 0);
    portEXIT_CRITICAL(&s_hx711_mux);

    esp_rom_delay_us(1);
}

esp_err_t hx711_init(hx711_t *dev, gpio_num_t dout, gpio_num_t sck, hx711_gain_t gain)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    dev->dout = dout;
    dev->sck  = sck;
    dev->gain = gain;

    gpio_config_t sck_cfg = {
        .pin_bit_mask = (1ULL << sck),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&sck_cfg);
    if (err != ESP_OK) return err;

    gpio_config_t dout_cfg = {
        .pin_bit_mask = (1ULL << dout),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&dout_cfg);
    if (err != ESP_OK) return err;

    /* Reset HX711 via power-down pulse (SCK HIGH > 60 us) then wake */
    gpio_set_level(sck, 1);
    esp_rom_delay_us(100);
    gpio_set_level(sck, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Discard initial conversion to latch channel/gain */
    int32_t dummy = 0;
    (void)hx711_read_raw(dev, &dummy, 400);
    return ESP_OK;
}

bool hx711_is_ready(const hx711_t *dev)
{
    return gpio_get_level(dev->dout) == 0;
}

bool hx711_is_saturated(int32_t raw)
{
    return (raw == 8388607) || (raw == -8388608);
}

esp_err_t hx711_read_raw(const hx711_t *dev, int32_t *out_raw, uint32_t timeout_ms)
{
    if (!dev || !out_raw) return ESP_ERR_INVALID_ARG;

    const int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
    while (gpio_get_level(dev->dout) != 0) {
        if (esp_timer_get_time() >= deadline_us) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    uint32_t raw = 0;
    for (int i = 0; i < 24; i++) {
        raw = (raw << 1) | (uint32_t)hx711_clock_bit(dev);
    }

    /* Extra clock pulses select channel & gain for the NEXT conversion */
    for (int i = 0; i < (int)dev->gain; i++) {
        hx711_clock_pulse(dev);
    }

    /* Sign-extend 24-bit two's complement to int32_t */
    if (raw & 0x00800000U) {
        raw |= 0xFF000000U;
    }
    *out_raw = (int32_t)raw;
    return ESP_OK;
}