#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * ============================================================================
 * drive_service.h - Module dong goi Vong dieu khien van toc ben trong (200 Hz)
 * ============================================================================
 *
 * VAI TRO TRONG KIEN TRUC PHAN TANG (Cascade - Hinh 6.9 & SMC_controller14.m):
 *
 *   [Vong ngoai: Luenberger 100Hz + SMC 20Hz + State Machine]
 *        |                                            ^
 *        | lenh (v_cmd, w_cmd)                        | trang thai (v_act, w_act,
 *        | + khoi luong m_load tu Loadcell HX711      |  s_enc, rpm_A/B, pwm_A/B)
 *        v                                            |
 *   +------------------------------------------------------------------------+
 *   |                      drive_service (Core 1, 200 Hz)                    |
 *   |  1. S-curve bac 3 (tuy chon khi khoi hanh muot, a_max <= 1.0 m/s^2)    |
 *   |  2. Dong hoc nghich vi sai: (v, w) -> (v_A_ref, v_B_ref)               |
 *   |  3. Hai bo 2-DOF PI doc lap + Luat tham so theo tai trong m_load       |
 *   |  4. Xuat PWM 12-bit Slow-Decay (motor.c) & Doc M/T 1x (encoder.c)      |
 *   |  5. Dong hoc thuan + Odometry khong troi: (N_A, N_B, v_A, v_B) ->      |
 *   |     (v_act, w_act, s_m)                                                |
 *   +------------------------------------------------------------------------+
 *
 * DAC DIEM AN TOAN & THOI GIAN THUC:
 *  - Chay doc quyen tren Core 1 bang ngat cung GPTimer 200 Hz (Ts = 5 ms),
 *    muc uu tien cao (priority 20), tuyet doi khong goi Wi-Fi / ESP_LOG ben
 *    trong vong 5 ms de giu jitter < 2 us.
 *  - Toan bo API goi tu Core 0 (app.c, SMC, Loadcell, Telemetry) deu duoc bao
 *    ve bang spinlock (portMUX_TYPE) -> thread-safe 100%.
 *  - Goi drive_service_lock_motors() NGAY DAU app_start() truoc ota_boot_window()
 *    de khoa cau H o trang thai Coast (Hi-Z) an toan trong suot thoi gian boot/OTA.
 * ============================================================================
 */

typedef enum {
    DRIVE_MODE_COAST = 0,       /* Ngat cau H (Hi-Z), xa tich phan PI - an toan khi cho */
    DRIVE_MODE_BRAKE,           /* Ham ngan mach Short Brake (IN1=IN2=1), xa tich phan PI */
    DRIVE_MODE_CHASSIS_VEL,     /* Dieu khien theo (v_cmd [m/s], w_cmd [rad/s]) tu vong ngoai */
    DRIVE_MODE_WHEEL_VEL        /* Dieu khien truc tiep tung banh (v_A_ref, v_B_ref) [m/s] */
} drive_mode_t;

/* Anh chup trang thai (thread-safe snapshot) vong trong cung cap cho vong ngoai */
typedef struct {
    drive_mode_t mode;          /* Che do hoat dong hien tai */

    /* Van toc thuc do tung banh (A = trai, B = phai) */
    float    v_a_mps;           /* Van toc dai banh trai [m/s] */
    float    v_b_mps;           /* Van toc dai banh phai [m/s] */
    float    rpm_a;             /* Toc do quay truc banh trai [RPM] */
    float    rpm_b;             /* Toc do quay truc banh phai [RPM] */

    /* Dong hoc thuan & Odometry cho Bo quan sat Luenberger & State Machine */
    float    v_mps;             /* Van toc dai tam xe v_act = 0.5*(v_A + v_B) [m/s] */
    float    w_rads;            /* Van toc goc than xe w_act = (v_B - v_A)/B [rad/s] */
    float    s_m;               /* Quang duong tich luy tu moc reset gan nhat [m] */

    /* Lenh dat & tin hieu dieu khien (phuc vu giam sat / telemetry) */
    float    v_cmd_eff_mps;     /* Van toc dai dat sau S-curve [m/s] */
    float    w_cmd_rads;        /* Van toc goc dat hien tai [rad/s] */
    float    v_a_ref_mps;       /* Van toc dai dat cho banh trai [m/s] */
    float    v_b_ref_mps;       /* Van toc dai dat cho banh phai [m/s] */
    float    pwm_a_pct;         /* PWM thuc xuat dong co A [%] */
    float    pwm_b_pct;         /* PWM thuc xuat dong co B [%] */

    /* Thong tin hieu chuan & thoi gian thuc */
    float    m_load_kg;         /* Khoi luong tai dang ap dung cho luat tham so [kg] */
    bool     scurve_active;     /* true neu dang trong pha tang toc muot S-curve */
    uint32_t tick_count;        /* Bo dem nhip 200 Hz */
    uint16_t dt_us_err;         /* Do lech chu ky |dt_actual - 5000| [us] */
} drive_state_t;

/*
 * ============================================================================
 * TRINH TU KHOI DONG AN TOAN (khuyen nghi trong app_start):
 *  1) Goi drive_service_lock_motors() NGAY DAU app_start() truoc ota_boot_window()
 *     de khoa cau H o trang thai Coast (Hi-Z) trong suot cua so OTA.
 *  2) Sau khi ota_boot_window() ket thuc, goi drive_service_start() de bat task
 *     dieu khien 200 Hz + ngat encoder + GPTimer doc quyen tren Core 1
 *     (tranh tranh chap ngat 200 Hz trong luc OTA ghi Flash).
 * ============================================================================
 */

/* Khoa an toan cau H o trang thai Coast (Hi-Z) ngay khi MCU vua khoi dong. */
void drive_service_lock_motors(void);

/*
 * Khoi chay task dieu khien vong trong 200 Hz ghim tren Core 1 (tu dong goi
 * drive_service_lock_motors() neu chua khoa truoc do).
 * Mac dinh nap luat tham so tren mat bat sa ban ung voi tai 0 kg.
 */
esp_err_t drive_service_start(void);

/* Ngat dong cau H (Coast / Hi-Z), huy S-curve va xoa tich phan PI. */
void drive_service_stop_coast(void);

/* Ham ngan mach chu dong (Short Brake), huy S-curve va xoa tich phan PI. */
void drive_service_stop_brake(void);

/*
 * Dat lenh van toc khung xe (v_mps, w_rads) tu bo dieu khien vong ngoai (SMC).
 *  - v_mps  : van toc dai mong muon [m/s]
 *  - w_rads : van toc goc mong muon [rad/s] (> 0 re trai / CCW)
 * Luu y: Neu S-curve dang chay, v_mps se bi gioi han boi bien do S-curve hien tai
 *        (min(v_mps, v_scurve)) de khong gay giat khi khoi hanh; nguoc lai neu
 *        vong ngoai giam toc/phanh (v_mps < v_scurve) thi dap ung giam ngay lap tuc.
 */
void drive_service_set_chassis_vel(float v_mps, float w_rads);

/*
 * Kich hoat quy dao khoi dong muot S-curve bac 3 tu van toc hien tai (hoac 0)
 * len v_target_mps trong khoang thoi gian duration_s [s].
 * Vi du chuan (Muc 6.1.1): drive_service_start_scurve(0.50f, 0.80f)
 * -> a_max = 0.9375 m/s^2 <= 1.0 m/s^2.
 * Trong qua trinh S-curve, vong ngoai van co the goi drive_service_set_chassis_vel()
 * de cap nhat w_rads va tran van toc v_mps (vi du v_prof giam toc).
 */
void drive_service_start_scurve(float v_target_mps, float duration_s);

/* Huy che do S-curve (chuyen sang bam truc tiep v_cmd). */
void drive_service_cancel_scurve(void);

/*
 * Dat truc tiep van toc dai [m/s] cho tung banh (dung cho test / hieu chuan).
 * Tu dong huy S-curve.
 */
void drive_service_set_wheel_vel(float v_a_mps, float v_b_mps);

/*
 * Cap nhat khoi luong tai hang m_load_kg [kg] do tu Loadcell HX711.
 * Ap dung luat tham so thich nghi tren mat bat sa ban (w_gc = 30 rad/s) o dau
 * chu ky 5 ms ke tiep.
 */
void drive_service_set_payload_kg(float m_load_kg);

/* Chuyen sang bo tham so ga do khong tai (w_gc = 35 rad/s, dung khi test tren gia do). */
void drive_service_use_bench_params(void);

/*
 * Dat lai moc quang duong odometry s_m = 0 (tuong duong `s_enc = 0` khi chuyen
 * sang chang 2 trong SMC_controller14.m) ma KHONG lam gian doan phep do van toc M/T.
 */
void drive_service_reset_odometry(void);

/* Doc anh chup trang thai dong bo (thread-safe) cua vong trong. */
void drive_service_get_state(drive_state_t *out);

/* Kiem tra ca 2 banh xe da dung han chua (|rpm_a| < 0.5 && |rpm_b| < 0.5). */
bool drive_service_is_stopped(void);

/*
 * Dang ky Task vong ngoai nhan thong bao dong bo pha tu nhip 200 Hz cua vong trong.
 *  - task_handle : handle cua task vong ngoai (truyen NULL de huy dang ky)
 *  - divider     : he so chia tan so (vd: 2 -> 100 Hz cho Observer, 10 -> 20 Hz cho SMC)
 */
void drive_service_register_outer_notify(TaskHandle_t task_handle, uint32_t divider);

/*
 * Ham tien ich tinh gia tri quy dao S-curve bac 3 thuan toan hoc:
 *   v(tau) = v_start + (v_target - v_start) * (3*tau^2 - 2*tau^3), tau in [0, 1].
 */
float drive_service_scurve_eval(float tau, float v_start, float v_target);
