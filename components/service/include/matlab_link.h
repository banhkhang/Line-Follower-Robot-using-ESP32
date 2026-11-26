#pragma once

/*
 * ============================================================================
 * matlab_link.h - Giao thuc Wi-Fi UDP 2 chieu giua ESP32 va MATLAB
 *                 (Dieu khien + Chinh tham so + Live Stream He A + Burst He B)
 * ============================================================================
 *
 * KHUNG GOI TIN NHI PHAN (Little-Endian):
 *   [0]     : 0xAA (Magic byte 1)
 *   [1]     : 0x55 (Magic byte 2)
 *   [2]     : pkt_type (uint8_t)
 *   [3..4]  : payload_len (uint16_t, LE)
 *   [5..5+N-1] : payload (N byte)
 *   [5+N]   : xor_cs = XOR cua cac byte tu [2] den [5+N-1]
 *
 * DANH SACH LENH TU MATLAB -> ESP32 (UDP Port 8889):
 *   0x01 CMD_PING        (0 B)  : Xac nhan ket noi / hoi trang thai -> tra PKT_STATUS
 *   0x02 CMD_START_A     (0 B)  : Chay He A (5S + live stream + dem nguoc 3s)
 *   0x03 CMD_START_B     (0 B)  : Chay He B (ACK -> tat Wi-Fi -> 7S + SRAM -> burst)
 *   0x04 CMD_STOP        (0 B)  : Dung khan cap (goi mission_ctrl_request_stop)
 *   0x05 CMD_SET_PARAMS  (52 B) : Cap nhat outer_smc_params_t (13 float)
 *   0x06 CMD_GET_PARAMS  (0 B)  : Hoi bo tham so SMC hien tai -> tra PKT_PARAMS
 *   0x07 CMD_BURST_REQ   (4 B)  : [uint16 start_chunk, uint16 count_chunks] (0,0 = gui tat ca)
 *   0x08 CMD_BURST_DONE  (0 B)  : Xac nhan MATLAB da nhan du burst -> giai phong SRAM
 *
 * DANH SACH GOI TU ESP32 -> MATLAB (UDP Port 8888):
 *   0x80 PKT_ACK         (2 B)  : [acked_cmd, err_code] (0 = OK)
 *   0x81 PKT_STATUS      (48 B) : [app_state, profile, calib_ok, sta_ok, robot_run_summary_t]
 *   0x82 PKT_PARAMS      (52 B) : outer_smc_params_t (13 float)
 *   0x83 PKT_LIVE_SAMPLE (52 B) : 1 mau robot_log_sample_t (He A)
 *   0x84 PKT_BURST_CHUNK (6+52*K): [chunk_idx(u16), total_chunks(u16), count(u16), samples[K]]
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "data_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_STATE_IDLE       = 0,  /* Cho lenh MATLAB hoac nut BOOT (Wi-Fi ON) */
    APP_STATE_RUNNING_A  = 1,  /* Dang chay He A (5S + Live Stream) */
    APP_STATE_RUNNING_B  = 2,  /* Dang chay He B (Wi-Fi OFF, khong gui qua mang) */
    APP_STATE_WAIT_BURST = 3,  /* He B da dung -> Wi-Fi ON, cho MATLAB PING & BURST_REQ */
    APP_STATE_CALIB      = 4   /* Dang hieu chuan 7 cam bien */
} app_link_state_t;

typedef enum {
    MATLAB_ACT_NONE       = 0,
    MATLAB_ACT_START_A    = 1,
    MATLAB_ACT_START_B    = 2,
    MATLAB_ACT_BURST_DONE = 3
} matlab_action_t;

/* Mo socket UDP va khoi chay task nhan lenh MATLAB tren Core 0 (khi Wi-Fi dang ON) */
esp_err_t matlab_link_start(void);

/* Dong socket & dung task sach se truoc khi tat Wi-Fi (de chuyen sang 7S / Calib) */
void      matlab_link_stop(void);

/* Cap nhat trang thai he thong de PKT_STATUS bao cao chinh xac */
void      matlab_link_set_app_state(app_link_state_t state);

/* Kiem tra MATLAB da tung ket noi (PING/CMD) hay chua */
bool      matlab_link_is_peer_known(void);

/* Lay va xoa hanh dong dang cho xu ly (START_A / START_B / BURST_DONE) */
matlab_action_t matlab_link_take_action(void);

/* Gui 1 mau live telemetry (52 byte) ve MATLAB trong He A */
void      matlab_link_send_live_sample(const robot_log_sample_t *sample);

/* Gui ngay goi PKT_STATUS (kem robot_run_summary_t) ve MATLAB */
void      matlab_link_send_status_now(void);

#ifdef __cplusplus
}
#endif
