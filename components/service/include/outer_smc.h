#pragma once

/*
 * ============================================================================
 * outer_smc.h - Bo dieu khien truot tich phan (Integral SMC), Lap lich chu ky
 *               thich nghi theo trang thai xac lap vong trong & Quy hoach van toc
 * ============================================================================
 *
 * NGUYEN LY BO DIEU KHIEN TRUOT TICH PHAN (Doria-Cerezo et al., ECC 2019):
 *  - Mat truot (Sliding Surface):
 *        sigma = e2 + beta_smc * e3_hat + z_int
 *    trong do:
 *        z_dot = ki_smc * e2   (khu sai so xac lap khi vao cua deu / lech 2 banh)
 *  - Luat dieu khien gom 2 thanh phan:
 *        u_eq  = [v_cmd * (sin(e3_hat) + beta_smc * c_hat) + ki_smc * e2] / (d1 + beta_smc)
 *        w_raw = u_eq + (k_smc / (d1 + beta_smc)) * sat(sigma / phi_bl)
 *    * u_eq (Dieu khien tuong duong): khu dong hoc lech goc va do cong quy dao.
 *    * k_smc * sat(sigma/phi_bl) (Dieu khien chuyen mach + Lop bien): keo trang
 *      thai ve mat truot trong thoi gian huu han nhung muot (chong rung chattering).
 *
 * NGUYEN LY LAP LICH CHU KY VONG NGOAI THICH NGHI (Ts_out in [0.050s .. 0.160s]):
 *  - Trong kien truc phan tang (cascade), vong ngoai chi nen cap nhat lenh moi
 *    khi vong trong da kip xac lap lenh truoc do de tranh dao dong pha.
 *  - Tuy nhien, thoi gian xac lap vong trong phu thuoc vao tai trong (0..2 kg)
 *    va bien do buoc nhay (buoc ±0.2 m/s mat 65..145 ms, con tinh chinh nho vai
 *    cm/s chi mat ~25..45 ms).
 *  - Do do, bo lap lich chay o nhip 100 Hz va kiem tra dong thoi 2 dieu kien:
 *      (1) Ca 2 banh da vao dai |v_ref - v_act| <= settle_band lien tuc du
 *          `settle_margin_s` (mac dinh 20 ms) VA da qua `ts_min_s` (>= 50 ms)
 *      (2) HOAC da cham tran an toan `ts_max_s` (mac dinh 160 ms, < 200 ms)
 *    => Phat lenh SMC moi ngay lap tuc va dung dung khoang thoi gian thuc `dt_smc`
 *       cho khau tich phan z_int!
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Tap tham so vong ngoai co the tinh chinh truc tiep tu MATLAB (CMD_SET_PARAMS) */
typedef struct {
    float v_req;            /* Van toc dai danh dinh [m/s] (mac dinh 0.50 m/s) */
    float beta_smc;         /* He so beta trong mat truot sigma [m/rad] (0.016) */
    float k_smc;            /* He so chuyen mach k > 0 [m/s] (0.30) */
    float phi_bl;           /* Do day lop bien chong rung [m] (0.015) */
    float ki_smc;           /* He so tich phan ki [1/s] (1.70) */
    float z_max;            /* Gioi han bao hoa tich phan z [m] (0.015) */
    float w_max;            /* Gioi han van toc goc dat [rad/s] (4.50) */

    /* Tham so lap lich chu ky thich nghi (0.05 s <= ts_min <= dt <= ts_max < 0.20 s) */
    float ts_min_s;         /* Chu ky toi thieu [s] (mac dinh 0.050 s) */
    float ts_max_s;         /* Chu ky toi da [s] (mac dinh 0.160 s) */
    float settle_margin_s;  /* Le tre sau khi 2 banh xac lap [s] (mac dinh 0.020 s) */
    float settle_band_mps;  /* Dai sai so xac lap moi banh [m/s] (mac dinh 0.020 m/s) */

    /* Vi chinh diem dung (trung diem truc banh chu dong) cho thuc nghiem [m] */
    float station_trim_m;   /* Bu tien/lui tai Tram tai hang [m] */
    float goal_trim_m;      /* Bu tien/lui tai Vach ket thuc [m] */
} outer_smc_params_t;

typedef struct {
    outer_smc_params_t p;

    /* Trang thai noi bo cua Integral SMC */
    float z_int;            /* Tich phan sai so e2 [m] */
    float sigma;            /* Gia tri mat truot gan nhat [m] */
    float sat_sigma;        /* Gia tri sat(sigma / phi_bl) in [-1, 1] */
    float w_cmd;            /* Lenh van toc goc giu mau (ZOH) hien tai [rad/s] */

    /* Trang thai bo lap lich thich nghi */
    float t_since_update_s; /* Thoi gian troi qua ke tu lan cap nhat SMC gan nhat [s] */
    float settle_dwell_s;   /* Thoi gian 2 banh da o trong dai xac lap lien tuc [s] */
    float last_dt_smc_s;    /* Chu ky thuc te cua lan cap nhat SMC gan nhat [s] */
    float last_e2_m;        /* Sai so e2 tai lan phat lenh SMC gan nhat (phat hien cat qua 0) */
} outer_smc_t;

/* Nap bo tham so mac dinh chuan tu robot_params.h */
void  outer_smc_get_default_params(outer_smc_params_t *out);

/* Khoi tao / dat lai trang thai SMC & bo lap lich */
void  outer_smc_init(outer_smc_t *smc, const outer_smc_params_t *params);
void  outer_smc_reset_state(outer_smc_t *smc);

/* Cap nhat tham so tu MATLAB (tu dong kep gioi han an toan) */
void  outer_smc_set_params(outer_smc_t *smc, const outer_smc_params_t *params);

/*
 * Kiem tra dieu kien lap lich va (neu den luot) tinh toan lenh van toc goc w_cmd moi.
 * Goi o moi nhip 100 Hz (dt_tick_s = 0.010 s):
 *  - e2_m, e3_hat_rad, c_hat : tu outer_estimator
 *  - v_cmd_mps               : van toc dai dat hien tai
 *  - seg_elapsed_s           : thoi gian da chay trong chang hien tai (de tre tich phan 0.3s)
 *  - va_ref, va_act, vb_ref, vb_act : van toc dat & thuc 2 banh tu drive_service
 *  - active_tracking         : true khi xe dang bam line (neu false -> w_cmd = 0)
 * Tra ve true neu SMC vua phat lenh moi trong nhip nay (de ghi co vao log).
 */
bool  outer_smc_step_adaptive(outer_smc_t *smc,
                              float e2_m,
                              float e3_hat_rad,
                              float c_hat,
                              float v_cmd_mps,
                              float seg_elapsed_s,
                              float va_ref_mps,
                              float va_act_mps,
                              float vb_ref_mps,
                              float vb_act_mps,
                              bool active_tracking,
                              bool line_ok,
                              float dt_tick_s);

/*
 * Quy hoach van toc giam toc muot ve v_min = 0.10 m/s truoc tram / dich:
 *   v(d_rem) = sqrt(v_min^2 + 2 * a_dec * (d_rem - d_margin))
 */
float outer_calc_decel_velocity(float d_rem_m, float v_req_mps, float v_min_mps,
                                float d_dec_m, float d_margin_m);

/*
 * Giam toc nhe dang ham cosine khi qua vung tach nhanh chu Y (s_enc in [1.85 .. 2.95] m)
 * de tang do chinh xac bam nhanh cua Gauss-Newton & Observer.
 */
float outer_calc_y_junction_speed(float s_enc_m, float v_req_mps);

/*
 * Tinh quang duong phanh du bao (braking lead) [m] cho Short Brake theo khoi luong tai
 * m_load_kg (noi suy tuyen tinh tu Bang 6.6: 64 / 71 / 78 mm o 0.70 m/s, ty le v^2
 * cong tre 1 chu ky vong trong 5 ms) de dung dung TRUNG DIEM TRUC BANH CHU DONG.
 */
float outer_estimate_brake_lead_m(float v_mps, float m_load_kg);

#ifdef __cplusplus
}
#endif
