#pragma once

/*
 * ============================================================================
 * outer_estimator.h - Bo loc Alpha-Beta, Tach nhanh Y Gauss-Newton &
 *                     Bo quan sat Luenberger thich nghi (Outer-Loop Estimator)
 * ============================================================================
 *
 * NGUYEN LY CHUNG & QUY UOC HE TRUC:
 *  - Chuyen doi tu file mo phong MATLAB `SMC_controller14.m` sang C nhung thoi
 *    gian thuc cho ESP32, su dung thong so chuan tai `robot_params.h`:
 *      * B  = 0.180 m (khoang cach 2 banh chu dong phia truoc)
 *      * d1 = 0.055 m (khoang cach truc banh chu dong -> day cam bien TCRT5000)
 *  - He truc xe chuan ISO (tay phai):
 *      * +X: huong xe tien ve phia truoc
 *      * +Y: sang TRAI -> sai so e2 > 0 khi vach line nam ben TRAI tam xe
 *      * +w: quay nguoc chieu kim dong ho (CCW = re TRAI)
 *    (Vi tren PCB, S1 nam ben TRAI ung voi -27 mm nen ben trong module nay toa
 *     do cam bien trong he SMC la y_sens_smc = [+27, +18, +9, 0, -9, -18, -27] mm.)
 *
 * KIEN TRUC 3 KHOI DA TAN SO:
 *  1) Bo loc Alpha-Beta (200 Hz ngoai vung chu Y, 100 Hz trong vung chu Y):
 *     Loc nhieu ADC cua sai so ngang e2 ma KHONG gay tre pha xac lap tren doan
 *     thang/cua deu (nho theo doi dong thoi vi tri e2 va van toc ngang de2).
 *  2) Bo tach nhanh chu Y Gauss-Newton (100 Hz):
 *     Tai nga ba chu Y, 2 nhanh cung xuat hien duoi day cam bien -> tinh trong
 *     tam thong thuong se bi keo vao khoang trang o giua. Thuat toan Gauss-Newton
 *     khop mau quang hoc 1 vach don 26 mm (ham tanh) ket hop danh trong so bat doi
 *     xung loai bo nhanh khong mong muon (1 kg = nhanh TRAI +Y, 2 kg = nhanh PHAI -Y).
 *  3) Bo quan sat Luenberger thich nghi (100 Hz, Ts = 10 ms):
 *     Uoc luong sai so goc lech huong e3_hat (khong do truc tiep duoc bang 1 day
 *     cam bien) va do cong quy dao c_hat tu (e2_filt, v_act, w_act). Tu dong tron
 *     cap cuc cham (loc nhieu tot) <-> cuc nhanh (bat kip chuyen tiep cua).
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "tcrt5000.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Trang thai bo loc Alpha-Beta cho sai so ngang e2 (he SMC, don vi [m]) */
    float e2_filt;          /* Sai so ngang da loc [m] (+ = line ben Trai) */
    float de2_filt;         /* Dao ham uoc luong de2/dt [m/s] */
    float alpha_sens;       /* He so alpha o tan so cam bien (200 Hz) */
    float beta_sens;        /* He so beta  o tan so cam bien (200 Hz) */
    float alpha_in;         /* He so alpha o tan so 100 Hz (trong vung chu Y) */
    float beta_in;          /* He so beta  o tan so 100 Hz (trong vung chu Y) */

    /* Trang thai Bo quan sat Luenberger thich nghi (100 Hz) */
    float e2_hat;           /* Uoc luong sai so ngang e2_hat [m] */
    float e3_hat;           /* Uoc luong sai so goc huong e3_hat [rad] */
    float c_hat;            /* Uoc luong do cong quy dao c_hat = w/v [1/m] */

    /* Co thong tin chan doan (phuc vu log/telemetry) */
    bool  y_gn_active;      /* true khi Gauss-Newton dang chu dong tach nhanh chu Y */
    float last_e2_raw;      /* Gia tri e2 tho gan nhat (hoac sau Gauss-Newton) [m] */
} outer_estimator_t;

/*
 * Khoi tao / dat lai bo loc va bo quan sat ve trang thai ban dau.
 *  - e2_init_m : sai so ngang ban dau [m] trong he SMC (+ = Trai)
 *  - ts_sens_s : chu ky doc cam bien [s] (mac dinh SENSOR_LOOP_TS_S = 0.005 s)
 *  - ts_in_s   : chu ky bo quan sat [s] (mac dinh OBSERVER_LOOP_TS_S = 0.010 s)
 */
void outer_estimator_reset(outer_estimator_t *est, float e2_init_m,
                           float ts_sens_s, float ts_in_s);

/*
 * Chi dat lai bo quan sat (giu nguyen he so bo loc) - goi khi roi Tram tai hang.
 */
void outer_estimator_reset_observer(outer_estimator_t *est, float e2_init_m);

/*
 * Buoc 1 (200 Hz): Du doan va cap nhat bo loc Alpha-Beta.
 *  - e2_raw_m  : sai so ngang do tu sensor_service (e2_smc_m) [m]
 *  - line_ok   : true neu day cam bien nhin thay line
 *  - in_y_zone : true neu robot dang o trong cua so nga ba chu Y
 *                (khi in_y_zone == true, chi du doan tuyen tinh o 200 Hz va
 *                 nhuong viec cap nhat cho Gauss-Newton o nhip 100 Hz)
 *  - ts_sens_s : chu ky 200 Hz (0.005 s)
 */
void outer_estimator_step_200hz(outer_estimator_t *est, float e2_raw_m,
                                bool line_ok, bool in_y_zone, float ts_sens_s);

/*
 * Buoc 2 (100 Hz - chi chay khi in_y_zone == true):
 * Chay thuat toan tach nhanh Gauss-Newton (3 vong lap) va cap nhat bo loc
 * Alpha-Beta theo nhip 100 Hz. Ho tro ca profile 7 mat (He B) va 5 mat (He A).
 *  - i_eff[7] : cuong do phan xa chuan hoa [0..1] theo dung thu tu vat ly S1..S7
 *  - total_i  : tong cuong do i_eff cua cac mat dang hoat dong
 *  - profile  : TCRT_PROFILE_5S hoac TCRT_PROFILE_7S
 *  - load_kg  : 1 (re nhanh TRAI / tren) hoac 2 (re nhanh PHAI / duoi)
 *  - ts_in_s  : chu ky 100 Hz (0.010 s)
 */
void outer_estimator_step_y_branch_100hz(outer_estimator_t *est,
                                         const float i_eff[LINE_SENSOR_MAX_COUNT],
                                         float total_i,
                                         tcrt_profile_t profile,
                                         uint8_t load_kg,
                                         float ts_in_s);

/*
 * Buoc 3 (100 Hz): Cap nhat Bo quan sat Luenberger thich nghi uoc luong (e2_hat, e3_hat, c_hat).
 *  - v_act_mps, w_act_rads : van toc dai & goc thuc do tu drive_service (Encoder M/T)
 *  - v_cmd_mps             : van toc dai dat hien tai (du phong khi |v_act| < 0.04 m/s)
 *  - active_tracking       : true khi xe dang bam line (MISSION_TO_LOAD hoac MISSION_TO_GOAL)
 *  - ts_in_s               : chu ky 100 Hz (0.010 s)
 */
void outer_estimator_step_observer_100hz(outer_estimator_t *est,
                                         float v_act_mps,
                                         float w_act_rads,
                                         float v_cmd_mps,
                                         bool active_tracking,
                                         bool line_ok,
                                         float ts_in_s);

#ifdef __cplusplus
}
#endif
