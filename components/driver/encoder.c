/*
 * encoder.c - Doc van toc M/T 1x (IRAM ISR Core 1) & bo dem xung Odometry
 *
 * Chi tiet nguyen ly va 4 co che bao ve xem trong encoder.h va Tai lieu Muc 5.2.1.c.
 */

#include "encoder.h"
#include "config.h"
#include "robot_params.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <math.h>
#include <stdbool.h>

/* Dam bao chan kenh B nam trong thanh ghi GPIO_IN_REG (GPIO 0..31) de doc 1 chu ky */
_Static_assert(PIN_ENC_A_B < 32 && PIN_ENC_B_B < 32,
               "Encoder channel B pins must be < 32 to use GPIO_IN_REG");

#define ENC_ZERO_TIMEOUT_US     25000LL /* 25 ms khong co xung -> 0 RPM (< 7.27 RPM) */
#define ENC_FIRST_EDGE_MAX_US   50000LL /* > 50 ms tu canh truoc -> dong bo moc, khong tinh gai */
#define ENC_GLITCH_GUARD_US     80LL    /* Loc nhieu EMI < 80 us (o 350 RPM, T_min = 519.5 us) */
#define ENC_SPIKE_MAX_DRPM      30.0f   /* Nguong chan gai mat 1 xung (~102 mm/s trong 5 ms) */

typedef struct {
    int   pin_a;
    int   pin_b;
    int   dir_sign;
    float ppr_out;          /* So canh len kenh A / vong banh xe = ENC_PPR_1X = 330 */

    portMUX_TYPE mux;
    volatile int32_t count;         /* Tong xung 1x tich luy co dau (cho M/T & odometry) */
    volatile int64_t last_edge_us;  /* Dau thoi gian canh len gan nhat [us] */

    int32_t prev_count;
    int64_t prev_edge_us;
    float   last_rpm;
    bool    spike_held;     /* Co danh dau da giu 1 mau de chan gai don le */
} enc_state_t;

static enc_state_t s_enc[2] = {
    {
        .pin_a    = PIN_ENC_A_A,
        .pin_b    = PIN_ENC_A_B,
        .dir_sign = ENC_DIR_A,
        .ppr_out  = ENC_PPR_1X,
        .mux      = portMUX_INITIALIZER_UNLOCKED
    },
    {
        .pin_a    = PIN_ENC_B_A,
        .pin_b    = PIN_ENC_B_B,
        .dir_sign = ENC_DIR_B,
        .ppr_out  = ENC_PPR_1X,
        .mux      = portMUX_INITIALIZER_UNLOCKED
    }
};

/*
 * Ngat canh len kenh A nam trong RAM noi (IRAM_ATTR):
 *  - Loc nhieu EMI (< 80 us so voi canh truoc).
 *  - Doc truc tiep muc logic kenh B tu thanh ghi GPIO_IN_REG de xac dinh chieu
 *    quay (cuc tinh b != 0 ? +1 : -1 dong bo voi ENC_DIR_A/B).
 *  - Luu dau thoi gian 1 us tu bo dem he thong esp_timer_get_time().
 */
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
    /* Kenh A: chi ngat canh len (POSEDGE) -> che do 1x (330 xung/vong) */
    gpio_config_t cfg_a = {
        .pin_bit_mask = (1ULL << PIN_ENC_A_A) | (1ULL << PIN_ENC_B_A),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE
    };
    gpio_config(&cfg_a);

    /* Kenh B: chi doc muc logic xac dinh chieu quay, khong bat ngat */
    gpio_config_t cfg_b = {
        .pin_bit_mask = (1ULL << PIN_ENC_A_B) | (1ULL << PIN_ENC_B_B),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&cfg_b);

    /* Cai dat dich vu ngat GPIO trong IRAM (bo qua neu da duoc cai truoc do) */
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

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
        e->spike_held   = false;
    }
}

int32_t encoder_get_count(enc_ch_t ch)
{
    enc_state_t *e = &s_enc[ch];
    int32_t cnt;

    portENTER_CRITICAL(&e->mux);
    cnt = e->count;
    portEXIT_CRITICAL(&e->mux);
    return cnt;
}

float encoder_sample_rpm(enc_ch_t ch, int64_t now_us, int16_t *out_delta)
{
    enc_state_t *e = &s_enc[ch];
    int32_t cnt;
    int64_t edge_us;

    /* Chup nguyen tu cap (so xung, moc canh cuoi) tu ISR */
    portENTER_CRITICAL(&e->mux);
    cnt     = e->count;
    edge_us = e->last_edge_us;
    portEXIT_CRITICAL(&e->mux);

    int32_t delta = cnt - e->prev_count;
    if (out_delta) {
        *out_delta = (int16_t)delta;
    }

    float prev_rpm = e->last_rpm;
    float rpm      = 0.0f;

    if (delta != 0) {
        if (e->prev_edge_us == 0 || (edge_us - e->prev_edge_us) > ENC_FIRST_EDGE_MAX_US) {
            /* Co che 2: Canh dau tien sau khi dung yen -> dong bo moc thoi gian */
            rpm = 0.0f;
        } else {
            /* Phep do M/T: chia cho khoang thoi gian thuc giua 2 canh len */
            int64_t dt_us = edge_us - e->prev_edge_us;
            if (dt_us > 0) {
                rpm = ((float)delta / e->ppr_out) * (60000000.0f / (float)dt_us);
            }
        }
        e->prev_count   = cnt;
        e->prev_edge_us = edge_us;
    } else {
        /* Co che 3: Khong co xung moi -> ep theo duong bao ly thuyet & timeout 25 ms */
        if (e->prev_edge_us == 0 || (now_us - e->prev_edge_us) >= ENC_ZERO_TIMEOUT_US) {
            rpm = 0.0f;
            e->prev_edge_us = 0;
        } else {
            float max_rpm = (1.0f / e->ppr_out) * (60000000.0f / (float)(now_us - e->prev_edge_us));
            if (fabsf(prev_rpm) > max_rpm) {
                rpm = copysignf(max_rpm, prev_rpm);
            } else {
                rpm = prev_rpm;
            }
        }
    }

    /*
     * Co che 4: Bo chan gai mat xung don le (|dn| > 30 RPM trong 5 ms ~ 20.4 m/s^2).
     * Neu phat hien buoc nhay bat kha thi vat ly: giu gia tri mau truoc dung 1 nhip
     * (khong gay tre pha dap ung binh thuong) va chap nhan gia tri moi o nhip ke tiep
     * neu bien thien do la that.
     */
    if (!e->spike_held && prev_rpm != 0.0f && fabsf(rpm - prev_rpm) > ENC_SPIKE_MAX_DRPM) {
        e->spike_held = true;
        return prev_rpm;
    }

    e->spike_held = false;
    e->last_rpm   = rpm;
    return rpm;
}
