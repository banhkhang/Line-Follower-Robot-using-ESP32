#pragma once

typedef enum {
    MOTOR_CH_A = 0,
    MOTOR_CH_B = 1
} motor_ch_t;

void motor_init(void);
void motor_coast(motor_ch_t ch);
void motor_brake(motor_ch_t ch);
void motor_stop_all_coast(void);
void motor_stop_all_brake(void);

/* Xuất PWM Slow-Decay: pwm_pct trong khoảng [-100.0, +100.0]
 * pwm_pct == 0.0 -> Short Brake (0%) */
void motor_set_pwm(motor_ch_t ch, float pwm_pct);