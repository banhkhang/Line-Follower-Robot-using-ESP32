#include "vel_ctrl.h"
#include <math.h>

#define V_REF_EPS       0.005f   /* Ngưỡng coi v_ref == 0 [m/s] */
#define V_REV_LOCK_THR  0.050f   /* Khóa biên dưới 0% khi v_ref > 0.05 m/s */

void vel_ctrl_init(vel_ctrl_t *c, const vel_ctrl_params_t *params)
{
    c->p = *params;
    vel_ctrl_reset(c);
}

void vel_ctrl_reset(vel_ctrl_t *c)
{
    c->integral = 0.0f;
    c->prev_err = 0.0f;
    c->in_band  = false;
}

float vel_ctrl_step(vel_ctrl_t *c, float v_ref, float v_meas, float dt_s)
{
    /* Khi đặt dừng hẳn: phanh Short Brake 0% và xả tích phân */
    if (fabsf(v_ref) <= V_REF_EPS) {
        c->integral = 0.0f;
        c->prev_err = 0.0f;
        c->in_band  = false;
        return 0.0f;
    }

    float e = v_ref - v_meas;

    /* 1. Feedforward + bù vùng chết ma sát tĩnh u_dz */
    float u_ff = c->p.kff * v_ref + copysignf(c->p.u_dz, v_ref);

    /* 2. Khâu tỉ lệ P */
    float u_p = c->p.kp * e;

    /* 3. Tích phân có điều kiện (Conditional Integration - Tustin) */
    bool  within_band = (fabsf(e) < c->p.e_band);
    float i_cand      = c->integral;
    if (within_band) {
        float e_prev_eff = c->in_band ? c->prev_err : e;
        i_cand = c->integral + (0.5f * c->p.ki * dt_s) * (e + e_prev_eff);
    }

    /* 4. Giới hạn động & Anti-windup Clamping bất đối xứng */
    float u_max = c->p.u_max;
    float u_min = (v_ref > V_REV_LOCK_THR) ? 0.0f : -u_max;
    if (v_ref >= 0.0f && u_min < 0.0f) {
        u_min = 0.0f; /* Không cấp áp ngược (plugging) khi chạy thuận */
    }

    float u_unsat = u_ff + u_p + i_cand;
    float u_out;

    if (u_unsat > u_max) {
        u_out = u_max;
        if (e < 0.0f && within_band) {
            c->integral = i_cand;
        }
    } else if (u_unsat < u_min) {
        u_out = u_min;
        if (e > 0.0f && within_band) {
            c->integral = i_cand;
        }
    } else {
        u_out = u_unsat;
        if (within_band) {
            c->integral = i_cand;
        }
    }

    c->prev_err = e;
    c->in_band  = within_band;
    return u_out;
}