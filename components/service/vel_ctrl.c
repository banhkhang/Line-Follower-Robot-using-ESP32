/*
 * vel_ctrl.c - Cai dat bo dieu khien van toc 2-DOF PI roi rac & luat tham so theo tai
 *
 * Chi tiet nguyen ly va cong thuc xem trong vel_ctrl.h va robot_params.h.
 */

#include "vel_ctrl.h"
#include "robot_params.h"
#include <math.h>
#include <stddef.h>

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
    /* Khi dat dung han (|v_ref| <= 5 mm/s): xuat Short Brake 0% va xa tich phan */
    if (fabsf(v_ref) <= VEL_CTRL_V_REF_EPS_MPS) {
        c->integral = 0.0f;
        c->prev_err = 0.0f;
        c->in_band  = false;
        return 0.0f;
    }

    float e = v_ref - v_meas;

    /* 1. Nhanh truyen thang kem bu vung chet ma sat tinh u_dz */
    float u_ff = c->p.kff * v_ref + copysignf(c->p.u_dz, v_ref);

    /* 2. Khau ti le P */
    float u_p = c->p.kp * e;

    /*
     * 3. Tich phan co dieu kien dang hinh thang Tustin:
     *    Chi cong don khi |e| < e_band. Tai mau dau tien vua buoc vao dai,
     *    gan e_prev_eff = e de khong keo theo sai so lon cua mau ngoai dai.
     */
    bool  within_band = (fabsf(e) < c->p.e_band);
    float i_cand      = c->integral;
    if (within_band) {
        float e_prev_eff = c->in_band ? c->prev_err : e;
        i_cand = c->integral + (0.5f * c->p.ki * dt_s) * (e + e_prev_eff);
    }

    /*
     * 4. Khoa bien duoi Anti-Plugging & Anti-windup Clamping bat doi xung:
     *    - Khi chay tien (v_ref >= 0), khoa u_min = 0% (Short Brake) de khong
     *      cap ap nguoc gay qua dong (V_nguon + E_emf) va tut lo.
     *    - Chi cho phep tich phan cap nhat khi khong bao hoa, hoac khi sai so
     *      doi dau theo chieu keo tin hieu thoat khoi vung bao hoa.
     */
    float u_max = c->p.u_max;
    float u_min = (v_ref > VEL_CTRL_REV_LOCK_THR_MPS) ? 0.0f : -u_max;
    if (v_ref >= 0.0f && u_min < 0.0f) {
        u_min = 0.0f;
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

void vel_ctrl_get_mat_params(float m_load_kg,
                             vel_ctrl_params_t *out_a,
                             vel_ctrl_params_t *out_b)
{
    if (m_load_kg < ROBOT_PAYLOAD_MIN_KG)       m_load_kg = ROBOT_PAYLOAD_MIN_KG;
    if (m_load_kg > ROBOT_PAYLOAD_CLAMP_MAX_KG) m_load_kg = ROBOT_PAYLOAD_CLAMP_MAX_KG;

    float eband = VEL_PI_MAT_EBAND0 + VEL_PI_MAT_EBAND_SLOPE * m_load_kg;

    if (out_a != NULL) {
        out_a->kp     = VEL_PI_MAT_A_KP0   + VEL_PI_MAT_A_KP_SLOPE  * m_load_kg;
        out_a->ki     = VEL_PI_MAT_A_KI;
        out_a->kff    = VEL_PI_MAT_A_KFF;
        out_a->u_dz   = VEL_PI_MAT_A_UDZ0  + VEL_PI_MAT_A_UDZ_SLOPE * m_load_kg;
        out_a->e_band = eband;
        out_a->u_max  = VEL_PI_U_MAX_PCT;
    }

    if (out_b != NULL) {
        out_b->kp     = VEL_PI_MAT_B_KP0   + VEL_PI_MAT_B_KP_SLOPE  * m_load_kg;
        out_b->ki     = VEL_PI_MAT_B_KI;
        out_b->kff    = VEL_PI_MAT_B_KFF;
        out_b->u_dz   = VEL_PI_MAT_B_UDZ0  + VEL_PI_MAT_B_UDZ_SLOPE * m_load_kg;
        out_b->e_band = eband;
        out_b->u_max  = VEL_PI_U_MAX_PCT;
    }
}

void vel_ctrl_get_bench_params(vel_ctrl_params_t *out_a,
                               vel_ctrl_params_t *out_b)
{
    if (out_a != NULL) {
        out_a->kp     = VEL_PI_BENCH_A_KP;
        out_a->ki     = VEL_PI_BENCH_A_KI;
        out_a->kff    = VEL_PI_BENCH_A_KFF;
        out_a->u_dz   = VEL_PI_BENCH_A_UDZ;
        out_a->e_band = VEL_PI_BENCH_EBAND;
        out_a->u_max  = VEL_PI_U_MAX_PCT;
    }

    if (out_b != NULL) {
        out_b->kp     = VEL_PI_BENCH_B_KP;
        out_b->ki     = VEL_PI_BENCH_B_KI;
        out_b->kff    = VEL_PI_BENCH_B_KFF;
        out_b->u_dz   = VEL_PI_BENCH_B_UDZ;
        out_b->e_band = VEL_PI_BENCH_EBAND;
        out_b->u_max  = VEL_PI_U_MAX_PCT;
    }
}

void vel_ctrl_apply_payload(vel_ctrl_t *ctrl_a, vel_ctrl_t *ctrl_b, float m_load_kg)
{
    vel_ctrl_get_mat_params(m_load_kg,
                            ctrl_a ? &ctrl_a->p : NULL,
                            ctrl_b ? &ctrl_b->p : NULL);
}
