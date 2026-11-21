#include "encoder.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <math.h>

#define ENC_ZERO_TIMEOUT_US   25000LL   /* 25 ms không có xung -> 0 RPM */
#define ENC_FIRST_EDGE_MAX_US 50000LL   /* > 50 ms từ cạnh trước -> đồng bộ mốc, bỏ qua gai */
#define ENC_GLITCH_GUARD_US   80LL      /* Lọc nhiễu EMI (< 80 us, tốc độ max 350 RPM có T=519 us) */

typedef struct {
    int   pin_a;
    int   pin_b;
    int   dir_sign;
    float ppr_out;      /* Số cạnh lên kênh A mỗi vòng bánh xe = ENC_CPR / 4 = 330 */

    portMUX_TYPE mux;
    volatile int32_t count;
    volatile int64_t last_edge_us;

    int32_t prev_count;
    int64_t prev_edge_us;
    float   last_rpm;
} enc_state_t;

static enc_state_t s_enc[2] = {
    {
        .pin_a = PIN_ENC_A_A, .pin_b = PIN_ENC_A_B,
        .dir_sign = ENC_DIR_A, .ppr_out = ENC_CPR_A * 0.25f,
        .mux = portMUX_INITIALIZER_UNLOCKED
    },
    {
        .pin_a = PIN_ENC_B_A, .pin_b = PIN_ENC_B_B,
        .dir_sign = ENC_DIR_B, .ppr_out = ENC_CPR_B * 0.25f,
        .mux = portMUX_INITIALIZER_UNLOCKED
    }
};

/* Ngắt cạnh lên kênh A: đọc nhanh chân B từ thanh ghi GPIO_IN_REG
 * Cực tính (b != 0 ? +1 : -1) giữ nguyên dấu với ENC_DIR_A/B của bản 4x trước đó */
static void IRAM_ATTR encoder_ch_a_isr(void *arg)
{
    enc_state_t *e = (enc_state_t *)arg;
    int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL_ISR(&e->mux);
    if (e->last_edge_us > 0 && (now_us - e->last_edge_us) < ENC_GLITCH_GUARD_US) {
        portEXIT_CRITICAL_ISR(&e->mux);
        return;
    }

    uint32_t in = REG_READ(GPIO_IN_REG);
    int8_t step = ((in >> e->pin_b) & 1U) ? +1 : -1;
    e->count += (step * e->dir_sign);
    e->last_edge_us = now_us;
    portEXIT_CRITICAL_ISR(&e->mux);
}

void encoder_init_on_core1(void)
{
    /* Kênh A: chỉ ngắt cạnh lên (POSEDGE) */
    gpio_config_t cfg_a = {
        .pin_bit_mask = (1ULL << PIN_ENC_A_A) | (1ULL << PIN_ENC_B_A),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE
    };
    gpio_config(&cfg_a);

    /* Kênh B: chỉ đọc mức logic để xác định chiều quay, không cần ngắt */
    gpio_config_t cfg_b = {
        .pin_bit_mask = (1ULL << PIN_ENC_A_B) | (1ULL << PIN_ENC_B_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg_b);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    for (int i = 0; i < 2; i++) {
        gpio_isr_handler_add(s_enc[i].pin_a, encoder_ch_a_isr, &s_enc[i]);
    }
    encoder_reset();
}

void encoder_reset(void)
{
    for (int i = 0; i < 2; i++) {
        enc_state_t *e = &s_enc[i];
        portENTER_CRITICAL(&e->mux);
        e->count        = 0;
        e->last_edge_us = 0;
        portEXIT_CRITICAL(&e->mux);

        e->prev_count   = 0;
        e->prev_edge_us = 0;
        e->last_rpm     = 0.0f;
    }
}

float encoder_sample_rpm(enc_ch_t ch, int64_t now_us, int16_t *out_delta)
{
    enc_state_t *e = &s_enc[ch];
    int32_t cnt;
    int64_t edge_us;

    portENTER_CRITICAL(&e->mux);
    cnt     = e->count;
    edge_us = e->last_edge_us;
    portEXIT_CRITICAL(&e->mux);

    int32_t delta = cnt - e->prev_count;
    if (out_delta) {
        *out_delta = (int16_t)delta;
    }

    float rpm = 0.0f;

    if (delta != 0) {
        if (e->prev_edge_us == 0 || (edge_us - e->prev_edge_us) > ENC_FIRST_EDGE_MAX_US) {
            /* Cạnh đầu tiên sau khi đứng yên: đồng bộ mốc thời gian, không xuất gai vận tốc */
            rpm = 0.0f;
        } else {
            int64_t dt_us = edge_us - e->prev_edge_us;
            if (dt_us > 0) {
                rpm = ((float)delta / e->ppr_out) * (60000000.0f / (float)dt_us);
            }
        }
        e->prev_count   = cnt;
        e->prev_edge_us = edge_us;
        e->last_rpm     = rpm;
    } else {
        if (e->prev_edge_us == 0 || (now_us - e->prev_edge_us) >= ENC_ZERO_TIMEOUT_US) {
            rpm = 0.0f;
            e->prev_edge_us = 0;
        } else {
            float max_rpm = (1.0f / e->ppr_out) * (60000000.0f / (float)(now_us - e->prev_edge_us));
            if (fabsf(e->last_rpm) > max_rpm) {
                rpm = copysignf(max_rpm, e->last_rpm);
            } else {
                rpm = e->last_rpm;
            }
        }
        e->last_rpm = rpm;
    }

    return rpm;
}