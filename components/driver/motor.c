/*
 * motor.c - Dieu khien dong co DC qua cau H DRV8871DDA & opto 6N137-300E
 *
 * Chi tiet xem bang chan tri trong motor.h va Tai lieu Chuong 5 (Bang 5.2).
 */

#include "motor.h"
#include "config.h"
#include "driver/ledc.h"
#include <math.h>

#define LEDC_MODE   LEDC_LOW_SPEED_MODE
#define LEDC_TIMER  LEDC_TIMER_0

/* Nap thanh ghi duty 12-bit (0..4095) co bao hoa bao ve. */
static inline void set_ch_duty(ledc_channel_t ch, uint32_t duty)
{
    if (duty > MOTOR_PWM_MAX_DUTY) {
        duty = MOTOR_PWM_MAX_DUTY;
    }
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

void motor_init(void)
{
    /*
     * f_PWM = 10 kHz (T = 100 us << hang so thoi gian dien L/R cua dong co),
     * APB 80 MHz / 10 kHz = 8000 > 4096 = 2^12 -> dat tron do phan giai 12-bit.
     */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = (ledc_timer_bit_t)MOTOR_PWM_RES_BITS,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    const int gpios[4] = {
        PIN_MOTOR_A1, PIN_MOTOR_A2,
        PIN_MOTOR_B1, PIN_MOTOR_B2
    };

    /*
     * Khoa cau H ngay khi khoi tao:
     * ESP32 xuat duty = MAX (muc HIGH) -> qua 6N137 dao thanh LOW -> DRV8871
     * nhan IN1=0, IN2=0 -> trang thai Coast (Hi-Z), cat dong hoan toan.
     */
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_cfg = {
            .speed_mode = LEDC_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = LEDC_TIMER,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = gpios[i],
            .duty       = MOTOR_PWM_MAX_DUTY,
            .hpoint     = 0
        };
        ledc_channel_config(&ch_cfg);
    }
}

void motor_coast(motor_ch_t ch)
{
    ledc_channel_t c1 = (ch == MOTOR_CH_A) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_2;
    ledc_channel_t c2 = (ch == MOTOR_CH_A) ? LEDC_CHANNEL_1 : LEDC_CHANNEL_3;
    /* GPIO = HIGH (4095/4095) -> qua 6N137 -> IN1/IN2 = 0/0 (Coast Hi-Z) */
    set_ch_duty(c1, MOTOR_PWM_MAX_DUTY);
    set_ch_duty(c2, MOTOR_PWM_MAX_DUTY);
}

void motor_brake(motor_ch_t ch)
{
    ledc_channel_t c1 = (ch == MOTOR_CH_A) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_2;
    ledc_channel_t c2 = (ch == MOTOR_CH_A) ? LEDC_CHANNEL_1 : LEDC_CHANNEL_3;
    /* GPIO = LOW (0/0) -> qua 6N137 -> IN1/IN2 = 1/1 (Short Brake) */
    set_ch_duty(c1, 0);
    set_ch_duty(c2, 0);
}

void motor_stop_all_coast(void)
{
    motor_coast(MOTOR_CH_A);
    motor_coast(MOTOR_CH_B);
}

void motor_stop_all_brake(void)
{
    motor_brake(MOTOR_CH_A);
    motor_brake(MOTOR_CH_B);
}

void motor_set_pwm(motor_ch_t ch, float pwm_pct)
{
    if (pwm_pct > 100.0f)  pwm_pct = 100.0f;
    if (pwm_pct < -100.0f) pwm_pct = -100.0f;

    ledc_channel_t c1 = (ch == MOTOR_CH_A) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_2;
    ledc_channel_t c2 = (ch == MOTOR_CH_A) ? LEDC_CHANNEL_1 : LEDC_CHANNEL_3;

    /* Quy doi truc tiep float [%] -> thanh ghi 12-bit [0..4095] (buoc nhay 0.0244%) */
    uint32_t duty = (uint32_t)lroundf((fabsf(pwm_pct) * 0.01f) * (float)MOTOR_PWM_MAX_DUTY);

    if (duty == 0) {
        /* 0% = Short Brake (IN1=1, IN2=1): tieu tan dong nang qua dien tro phan ung */
        set_ch_duty(c1, 0);
        set_ch_duty(c2, 0);
    } else if (pwm_pct > 0.0f) {
        /* Tien (Slow Decay): IN1=1 (GPIO1=0), IN2 nhan PWM dao (GPIO2=duty) */
        set_ch_duty(c1, 0);
        set_ch_duty(c2, duty);
    } else {
        /* Lui  (Slow Decay): IN1 nhan PWM dao (GPIO1=duty), IN2=1 (GPIO2=0) */
        set_ch_duty(c1, duty);
        set_ch_duty(c2, 0);
    }
}
