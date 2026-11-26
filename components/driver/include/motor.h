#pragma once

/*
 * ============================================================================
 * motor.h - Trinh dieu khien cau H DRV8871 qua opto cach ly 6N137 (LEDC PWM)
 * ============================================================================
 *
 * NGUYEN LY PHAN CUNG & CHE DO DIEU CHE:
 *  - Tin hieu PWM 10 kHz, do phan giai 12-bit (0..4095) tu ESP32 di qua 4 opto
 *    cach ly toc do cao 6N137-300E (DAO PHA LOGIC) truoc khi vao IN1/IN2 cua
 *    mach cau H DRV8871DDA.
 *  - De bao toan tinh tuyen tinh cua dac tinh van toc - PWM, mach chay o che
 *    do SLOW DECAY (luan phien Drive va Short Brake):
 *      + Coast (Hi-Z, troi tu do): GPIO1=4095, GPIO2=4095 -> IN1=0, IN2=0
 *      + Short Brake (ham cung)  : GPIO1=0,    GPIO2=0    -> IN1=1, IN2=1
 *      + Tien (pwm_pct > 0)      : GPIO1=0,    GPIO2=duty -> IN1=1, IN2=PWM dao
 *      + Lui  (pwm_pct < 0)      : GPIO1=duty, GPIO2=0    -> IN1=PWM dao, IN2=1
 *  - Viec quy doi truc tiep tu float [-100.0, +100.0] sang thanh ghi 12-bit
 *    giup giam sai so luong tu hoa van toc xuong chi ~0.30 mm/s (loai bo dao
 *    dong gioi han do luong tu hoa co cau chap hanh).
 *
 * QUY UOC KENH TREN ROBOT VI SAI:
 *  - MOTOR_CH_A (MOTOR_CH_LEFT)  : Banh chu dong ben TRAI
 *  - MOTOR_CH_B (MOTOR_CH_RIGHT) : Banh chu dong ben PHAI
 * ============================================================================
 */

typedef enum {
    MOTOR_CH_A = 0,                 /* Dong co A - banh TRAI */
    MOTOR_CH_B = 1,                 /* Dong co B - banh PHAI */
    MOTOR_CH_LEFT  = MOTOR_CH_A,    /* Bi danh ngu nghia cho dong hoc vi sai */
    MOTOR_CH_RIGHT = MOTOR_CH_B
} motor_ch_t;

/* Khoi tao bo dinh thoi & 4 kenh LEDC 10 kHz 12-bit, khoa an toan o trang thai Coast (Hi-Z). */
void motor_init(void);

/* Ngat hai nhanh cau H (Hi-Z, roto troi tu do). Dung khi boot, cho hoac cat dong an toan. */
void motor_coast(motor_ch_t ch);

/* Ngan mach hai dau dong co (Short Brake, IN1=IN2=1). Dung de ham chu dong khi dung. */
void motor_brake(motor_ch_t ch);

/* Ngat dong (Coast) dong thoi ca 2 dong co. */
void motor_stop_all_coast(void);

/* Ham ngan mach (Short Brake) dong thoi ca 2 dong co. */
void motor_stop_all_brake(void);

/*
 * Xuat xung PWM Slow-Decay cho dong co.
 *  - pwm_pct : do rong xung trong khoang [-100.0f, +100.0f] [%]
 *              > 0: quay tien, < 0: quay lui, == 0 (hoac lam tron ve 0): Short Brake.
 */
void motor_set_pwm(motor_ch_t ch, float pwm_pct);
