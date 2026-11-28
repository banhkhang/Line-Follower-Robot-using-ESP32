#pragma once

/*
 * ============================================================================
 * mission_ctrl.h - Dieu phoi Vong ngoai (Core 0, dong bo 200 Hz voi Core 1),
 *                  May trang thai nhiem vu & Watchdog dung an toan
 *                  (Phien ban doc lap khong Wi-Fi / khong log)
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include "outer_smc.h"
#include "tcrt5000.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MISSION_IDLE            = 0,  /* Cho lenh nut BOOT */
    MISSION_TO_LOAD         = 1,  /* Chang 1: Xuat phat (3000) -> Vach ngang tram tai (1000) */
    MISSION_LOAD_AXLE_ALIGN = 2,  /* Tien them 100 mm sau tam vach ngang vao vung tai hang */
    MISSION_WAIT_LOAD       = 3,  /* Dung Short Brake, doc HX711 cho hang > 700g on dinh 1.2s */
    MISSION_TO_GOAL         = 4,  /* Chang 2: Nga ba Y (1/2 kg) + U-turn R500 -> Can dich */
    MISSION_GOAL_AXLE_ALIGN = 5,  /* Cam bien cham het vach -> khoa thang dua TRUC BANH vao dich */
    MISSION_STOPPED_GOAL    = 6,  /* Da dung han tai dich, dem du 2.0 s roi ket thuc */
    MISSION_STOPPED_FAULT   = 7   /* Dung do mat line 2s / |e2|>16mm 2s / nhan BOOT dung khan */
} mission_state_t;

typedef enum {
    STOP_REASON_NONE        = 0,
    STOP_REASON_GOAL_DONE   = 1,  /* Hoan thanh nhiem vu & dung tai dich du 2.0 s */
    STOP_REASON_LINE_LOST_2S= 2,  /* Mat line lien tuc >= 2.0 s */
    STOP_REASON_ERR_16MM_2S = 3,  /* |e2| > 16 mm lien tuc >= 2.0 s */
    STOP_REASON_MANUAL      = 4,  /* Nguoi dung nhan giu nut BOOT de dung */
    STOP_REASON_FAULT       = 5   /* Loi doc cam bien / tao task */
} mission_stop_reason_t;

typedef struct {
    tcrt_profile_t sensor_profile;   /* Profile cam bien su dung (7S hoac 5S) */
    uint32_t       countdown_rem_ms; /* Thoi gian dem nguoc truoc khi khoi hanh [ms] */
} mission_run_cfg_t;

/* Khoi tao module (nap bo tham so SMC mac dinh tu robot_params.h) */
void mission_ctrl_init(void);

/*
 * Chay tron ven 1 luot nhiem vu bam line & tai hang (blocking den khi dung han).
 *  - Thuc hien dem nguoc `countdown_rem_ms` voi LED1 nhap nhay (nhan BOOT de huy).
 *  - Trong luc chay: giu BOOT >= 150 ms se phanh Short Brake khan cap ngay.
 */
mission_stop_reason_t mission_ctrl_run_blocking(const mission_run_cfg_t *cfg);

bool            mission_ctrl_is_running(void);
mission_state_t mission_ctrl_get_state(void);

#ifdef __cplusplus
}
#endif
