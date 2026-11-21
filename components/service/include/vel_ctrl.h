#pragma once
#include <stdbool.h>

typedef struct {
    float kp;      /* [%PWM / (m/s)] */
    float ki;      /* [%PWM / m]     */
    float kff;     /* [%PWM / (m/s)] */
    float u_dz;    /* [%PWM] vùng chết ma sát tĩnh */
    float e_band;  /* [m/s] dải sai số kích hoạt tích phân Tustin */
    float u_max;   /* [%PWM] giới hạn trên (100.0) */
} vel_ctrl_params_t;

typedef struct {
    vel_ctrl_params_t p;
    float integral;
    float prev_err;
    bool  in_band;
} vel_ctrl_t;

void  vel_ctrl_init(vel_ctrl_t *c, const vel_ctrl_params_t *params);
void  vel_ctrl_reset(vel_ctrl_t *c);
float vel_ctrl_step(vel_ctrl_t *c, float v_ref, float v_meas, float dt_s);