#include <stddef.h>
#include <stdbool.h>
#include "hx711.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static gpio_num_t s_dout = GPIO_NUM_NC;
static gpio_num_t s_sck  = GPIO_NUM_NC;
static bool s_ready_to_use = false;

/* Protects the bit-banging from interrupts (see hx711_read_raw). */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

esp_err_t hx711_init(gpio_num_t dout_pin, gpio_num_t sck_pin)
{
    gpio_config_t dout_cfg = {
        .pin_bit_mask = 1ULL << dout_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config_t sck_cfg = {
        .pin_bit_mask = 1ULL << sck_pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&dout_cfg);
    if (err != ESP_OK) return err;
    err = gpio_config(&sck_cfg);
    if (err != ESP_OK) return err;

    s_dout = dout_pin;
    s_sck  = sck_pin;

    /* SCK must idle LOW. If it stays HIGH for more than 60 us the chip powers down. */
    gpio_set_level(s_sck, 0);

    s_ready_to_use = true;
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
    return s_ready_to_use && gpio_get_level(s_dout) == 0;
}

esp_err_t hx711_read_raw(int32_t *value, uint32_t timeout_ms)
{
    if (!s_ready_to_use || value == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 1. Wait until the chip says "sample ready" by pulling DOUT low. */
    TickType_t start = xTaskGetTickCount();
    while (gpio_get_level(s_dout) != 0) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }

    /* 2. Clock out 24 bits, most significant first.
     *    Interrupts are blocked for the ~100 us this takes, so nothing can
     *    stretch a clock pulse and accidentally power the chip down. */
    uint32_t data = 0;

    portENTER_CRITICAL(&s_lock);
    for (int i = 0; i < 24; i++) {
        gpio_set_level(s_sck, 1);
        esp_rom_delay_us(1);
        data = (data << 1) | (uint32_t)gpio_get_level(s_dout);
        gpio_set_level(s_sck, 0);
        esp_rom_delay_us(1);
    }

    /* 3. One extra pulse tells the chip: next time use channel A, gain 128. */
    gpio_set_level(s_sck, 1);
    esp_rom_delay_us(1);
    gpio_set_level(s_sck, 0);
    esp_rom_delay_us(1);
    portEXIT_CRITICAL(&s_lock);

    /* 4. The number is 24-bit two's complement: extend the sign to 32 bits. */
    if (data & 0x800000) {
        data |= 0xFF000000;
    }
    *value = (int32_t)data;

    return ESP_OK;
}
