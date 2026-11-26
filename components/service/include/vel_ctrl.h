#pragma once

#include <stdbool.h>

/*
 * ============================================================================
 * vel_ctrl.h - Bo dieu khien van toc 2-DOF PI roi rac + Luat tham so theo tai
 * ============================================================================
 *
 * NGUYEN LY DIEU KHIEN (Muc 6.1.1.c & 6.1.3 - Hinh 6.3):
 *  Day la khoi toan hoc thuan (khong phu thuoc RTOS/phan cung) thuc thi bo
 *  dieu khien van toc vong trong 2 bac tu do (2-DOF PI) cho 1 dong co DC:
 *
 *   1) Truyen thang kem bu vung chet (Feedforward + Deadzone):
 *        u_ff[k] = K_ff * v_ref[k] + u_dz * sign(v_ref[k])  khi |v_ref| > 5 mm/s
 *        u_ff[k] = 0 (Short Brake, xoa tich phan)           khi |v_ref| <= 5 mm/s
 *      Giup nhanh truyen thang gan nhu tao san muc PWM xac lap, de bo PI chi
 *      can khu sai lech dong nho.
 *
 *   2) Khau ti le P + Tich phan co dieu kien dang Tustin:
 *        u_p[k]    = K_p * e[k],   voi e[k] = v_ref[k] - v_meas[k]
 *        I_cand[k] = I[k-1] + 0.5 * K_i * Ts * (e[k] + e_prev_eff[k])
 *      Khau tich phan chi duoc phep cong don khi sai so da lot vao dai nho
 *      |e[k]| < e_band (tranh tich luy thua khi tang/giam toc gay vot lo).
 *
 *   3) Khoa bien duoi chong cap ap nguoc (Anti-Plugging):
 *      Khi xe dang chay tien (v_ref > 0.05 m/s) ma giam toc, bien duoi duoc
 *      khoa tai u_min = 0% (phanh ngan mach Short Brake) de khong cap ap nguoc
 *      gay qua dong (V_nguon + E_emf) va tut lo manh.
 *
 *   4) Chong bao hoa tich phan bat doi xung (Anti-windup Clamping):
 *      Khi u_unsat cham bien [u_min, u_max], giu nguyen tich phan va chi cap
 *      nhat tro lai khi sai so doi dau theo chieu thoat bao hoa.
 *
 *   5) Luat tham so thich nghi theo khoi luong tai hang m_load [kg]:
 *      Khi cho hang 0..2 kg tren mat bat, hang so thoi gian co hoc tau_m va
 *      vung chet ma sat lan u_dz tang tuyen tinh theo m_load. Ham
 *      vel_ctrl_apply_payload() cap nhat truc tiep (Kp, u_dz, e_band) theo
 *      ket qua hieu chuan 3 muc tai (0/1/2 kg) voi tan so cat w_gc = 30 rad/s.
 * ============================================================================
 */

typedef struct {
    float kp;      /* Do loi ti le [%PWM / (m/s)] */
    float ki;      /* Do loi tich phan [%PWM / m] */
    float kff;     /* Do loi truyen thang [%PWM / (m/s)] */
    float u_dz;    /* Nguong bu vung chet ma sat tinh [%PWM] */
    float e_band;  /* Dai sai so kich hoat tich phan Tustin [m/s] */
    float u_max;   /* Gioi han bao hoa tren PWM (thuong = 100.0 %) */
} vel_ctrl_params_t;

typedef struct {
    vel_ctrl_params_t p;    /* Bo tham so hien hanh */
    float integral;         /* Trang thai khau tich phan I[k-1] [%PWM] */
    float prev_err;         /* Sai so mau truoc e[k-1] [m/s] */
    bool  in_band;          /* Co cho biet mau truoc da o trong dai |e| < e_band */
} vel_ctrl_t;

/* Khoi tao bo dieu khien voi bo tham so cho truoc va xoa trang thai noi bo. */
void vel_ctrl_init(vel_ctrl_t *c, const vel_ctrl_params_t *params);

/* Xoa tich phan & lich su sai so ve 0 (goi khi dung/chuyen che do). */
void vel_ctrl_reset(vel_ctrl_t *c);

/*
 * Thuc thi 1 buoc dieu khien 2-DOF PI roi rac.
 *  - v_ref  : van toc dai dat [m/s]
 *  - v_meas : van toc dai do duoc tu encoder M/T [m/s]
 *  - dt_s   : chu ky lay mau Ts [s] (mac dinh 0.005 s tai 200 Hz)
 *  - Tra ve : lenh do rong xung PWM [%], trong khoang [u_min, u_max].
 */
float vel_ctrl_step(vel_ctrl_t *c, float v_ref, float v_meas, float dt_s);

/*
 * Tinh bo tham so 2-DOF PI tren mat bat sa ban theo khoi luong tai m_load_kg [0..2.5 kg]
 * (w_gc = 30 rad/s, PM ~ 72.8..76.2 deg, hieu chuan 3 moc 0/1/2 kg - Bang 6.6).
 */
void vel_ctrl_get_mat_params(float m_load_kg,
                             vel_ctrl_params_t *out_a,
                             vel_ctrl_params_t *out_b);

/*
 * Lay bo tham so 2-DOF PI danh cho ga do khong tai (w_gc = 35 rad/s - Bang 6.3).
 */
void vel_ctrl_get_bench_params(vel_ctrl_params_t *out_a,
                               vel_ctrl_params_t *out_b);

/*
 * Ap dung truc tiep luat tham so theo tai trong m_load_kg [kg] cho ca 2 kenh
 * dieu khien A (trai) va B (phai) ma khong xoa trang thai tich phan dang chay
 * (cho phep cap nhat muot tu Loadcell HX711).
 */
void vel_ctrl_apply_payload(vel_ctrl_t *ctrl_a, vel_ctrl_t *ctrl_b, float m_load_kg);
