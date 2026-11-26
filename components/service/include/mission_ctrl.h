#pragma once

/*
 * ============================================================================
 * mission_ctrl.h - Dieu phoi Vong ngoai (Core 0, dong bo 200 Hz voi Core 1),
 *                  May trang thai nhiem vu & Watchdog dung an toan
 * ============================================================================
 *
 * LUONG HOAT DONG TONG THE:
 *  - Task vong ngoai `outer_ctrl_task` chay tren Core 0, nhan bao thuc dong bo
 *    pha moi 5 ms (200 Hz) tu GPTimer cua `drive_service` (Core 1) -> KHONG tao
 *    them ngat phan cung moi.
 *  - Moi 5 ms (200 Hz):
 *      + Doc TCRT5000 (ambient-compensated, oversample 2x, ~0.75 ms)
 *      + Loc Alpha-Beta + phat hien vach cat ngang tram tai / dau mut ket thuc
 *      + Kiem tra 3 dieu kien dung an toan:
 *          (1) Mat line lien tuc >= 2.0 s
 *          (2) Sai so |e2| > 16 mm lien tuc >= 2.0 s
 *          (3) Dung tai vach dich du 2.0 s
 *  - Moi 10 ms (100 Hz, divider 2):
 *      + Tach nhanh chu Y Gauss-Newton (khi trong cua so odometry [2.05..2.70] m)
 *      + Bo quan sat Luenberger thich nghi (uoc luong e3_hat, c_hat)
 *      + May trang thai nhiem vu (Tram tai hang -> Can HX711 -> Re nhanh 1/2 kg ->
 *        Can chinh TRUC BANH CHU DONG dung +-5 mm tai tram & dich)
 *      + Bo lap lich SMC thich nghi (cap nhat w_cmd trong [50..160] ms tuy theo
 *        vong trong da xac lap hay chua)
 *      + Gui live sample (He A) hoac ghi vao SRAM `data_logger` (He B)
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "data_logger.h"
#include "outer_smc.h"
#include "tcrt5000.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MISSION_IDLE            = 0,  /* Cho lenh */
    MISSION_TO_LOAD         = 1,  /* Chang 1: Xuat phat (3000) -> Vach ngang tram tai (1000) */
    MISSION_LOAD_AXLE_ALIGN = 2,  /* Tien them ~d1 sau vach ngang de TRUC BANH dung dung tam tram */
    MISSION_WAIT_LOAD       = 3,  /* Dung Short Brake, doc HX711 cho hang > 700g on dinh 1.2s */
    MISSION_TO_GOAL         = 4,  /* Chang 2: U-turn R500 + Nga ba Y (1/2 kg) -> Can dich */
    MISSION_GOAL_AXLE_ALIGN = 5,  /* Cam bien vua cham het vach -> khoa thang bo ~d1 dua TRUC BANH vao dich */
    MISSION_STOPPED_GOAL    = 6,  /* Da dung han tai dich, dem du 2.0 s roi ket thuc */
    MISSION_STOPPED_FAULT   = 7   /* Dung do mat line 2s / |e2|>16mm 2s / lenh dung khan cap */
} mission_state_t;

/* Callback gui mau truc tiep cho He A (goi o 100 Hz / SYSTEM_A_LIVE_DIV) */
typedef void (*mission_live_sample_cb_t)(const robot_log_sample_t *sample);

typedef struct {
    robot_run_mode_t         mode;              /* A / B / Standalone 7S / Standalone 5S */
    tcrt_profile_t           sensor_profile;    /* 5 hoac 7 cam bien */
    bool                     enable_sram_log;   /* true cho He B */
    mission_live_sample_cb_t live_cb;           /* != NULL cho He A */
    uint32_t                 countdown_rem_ms;  /* Thoi gian dem nguoc con lai truoc khi chay */
} mission_run_cfg_t;

/* Khoi tao module (nap tham so SMC mac dinh) */
void mission_ctrl_init(void);

/* Cap nhat / doc bo tham so SMC hien tai (thread-safe, dung cho CMD_SET_PARAMS) */
void mission_ctrl_set_params(const outer_smc_params_t *params);
void mission_ctrl_get_params(outer_smc_params_t *out);

/*
 * Chay tron ven 1 luot nhiem vu (blocking cho den khi xe dung han do 1 trong 3
 * dieu kien [Stop] hoac dung thu cong).
 *  - Tu dong thuc hien dem nguoc `countdown_rem_ms` voi nhay LED1 canh bao.
 *  - Co the dung khan cap tu task khac (MATLAB STOP) bang `mission_ctrl_request_stop()`,
 *    hoac bang cach nhan nut BOOT tren xe.
 *  - Khi ket thuc, tra ve `out_summary` (neu != NULL) va khoa dong co an toan (Coast).
 */
robot_stop_reason_t mission_ctrl_run_blocking(const mission_run_cfg_t *cfg,
                                              robot_run_summary_t *out_summary);

/* Yeu cau dung khan cap tu xa (thread-safe, goi tu MATLAB task) */
void mission_ctrl_request_stop(void);

/* Kiem tra co dang chay nhiem vu khong */
bool mission_ctrl_is_running(void);
mission_state_t mission_ctrl_get_state(void);

#ifdef __cplusplus
}
#endif
