#pragma once

/*
 * ============================================================================
 * data_logger.h - Bo ghi du lieu nhi phan vao SRAM noi (System B) &
 *                 Cau truc goi du lieu dung chung voi Live Stream (System A)
 * ============================================================================
 *
 * NGUYEN LY QUAN LY BO NHO SRAM TREN ESP32-WROOM-32E (KHONG CO PSRAM):
 *  - Moi mau `robot_log_sample_t` co kich thuoc nen dung 52 byte, chua du toan
 *    bo tin hieu can thiet de hieu chinh ca vong ngoai (e2_raw, e2_filt, e3_hat,
 *    c_hat, sigma, z_int, dt_smc, 7 mat cam bien) lan vong trong (v_A/B ref/act,
 *    PWM A/B, odometry s_enc, jitter) va danh gia sai so bam line / sai so dung.
 *  - Thay vi cap phat 1 khoi lien tuc lon (de bi phan manh heap), bo logger cap
 *    phat theo tung khoi nho 200 mau (~10.4 KB/khoi) va LUON giu lai it nhat
 *    85 KB heap DRAM trong de dam bao `esp_wifi_init()` / `esp_wifi_start()`
 *    luon khoi dong thanh cong khi xe dung va chuyen sang pha Burst Wi-Fi!
 * ============================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RUN_MODE_IDLE          = 0,
    RUN_MODE_SYS_A_5S_LIVE = 1,   /* He A: 5 cam bien + Wi-Fi live stream MATLAB */
    RUN_MODE_SYS_B_7S_BURST= 2,   /* He B: 7 cam bien + log SRAM + burst Wi-Fi sau khi dung */
    RUN_MODE_STANDALONE_7S = 3,   /* Doc lap bang nut BOOT (1 click): 7 cam bien, khong log */
    RUN_MODE_STANDALONE_5S = 4    /* Doc lap bang nut BOOT (2 click): 5 cam bien, khong log */
} robot_run_mode_t;

typedef enum {
    STOP_REASON_NONE        = 0,
    STOP_REASON_GOAL_DONE   = 1,  /* Dung tai vach dich du 2.0 s */
    STOP_REASON_LINE_LOST_2S= 2,  /* Mat line lien tuc >= 2.0 s */
    STOP_REASON_ERR_16MM_2S = 3,  /* Sai so |e2| > 16 mm lien tuc >= 2.0 s */
    STOP_REASON_MANUAL      = 4,  /* Dung thu cong tu MATLAB hoac nut BOOT */
    STOP_REASON_FAULT       = 5   /* Loi phan cung / cam bien */
} robot_stop_reason_t;

/* Co bit trong truong `state_flags` cua robot_log_sample_t */
#define LOG_FLAG_STATE_MASK     0x0FU   /* bits [3:0] = mission_state_t */
#define LOG_FLAG_LINE_OK        (1U << 4)
#define LOG_FLAG_Y_GN_ACTIVE    (1U << 5)
#define LOG_FLAG_SMC_UPDATED    (1U << 6)
#define LOG_FLAG_MARKER_SEEN    (1U << 7)

/* 1 mau log nhi phan nen = dung 52 byte (Little-Endian, khop 1:1 voi MATLAB) */
typedef struct __attribute__((packed)) {
    uint32_t t_ms;              /* [0..3]   Thoi gian tinh tu luc bat dau chay [ms] */
    int16_t  e2_raw_cmm;        /* [4..5]   Sai so ngang tho / Gauss-Newton [0.01 mm] (+ = Trai) */
    int16_t  e2_filt_cmm;       /* [6..7]   Sai so ngang sau bo loc Alpha-Beta [0.01 mm] */
    int16_t  e3_hat_cdeg;       /* [8..9]   Goc lech uoc luong Luenberger [0.01 deg] */
    int16_t  c_hat_mrad;        /* [10..11] Do cong quy dao uoc luong [0.001 rad/m] */
    int16_t  sigma_cmm;         /* [12..13] Mat truot SMC sigma [0.01 mm] */
    int16_t  z_int_cmm;         /* [14..15] Thanh phan tich phan SMC z_int [0.01 mm] */
    int16_t  v_cmd_mms;         /* [16..17] Van toc dai dat v_cmd [mm/s] */
    int16_t  w_cmd_mrads;       /* [18..19] Van toc goc dat w_cmd [mrad/s] */
    int16_t  v_act_mms;         /* [20..21] Van toc dai thuc do v_act [mm/s] */
    int16_t  w_act_mrads;       /* [22..23] Van toc goc thuc do w_act [mrad/s] */
    int16_t  va_ref_mms;        /* [24..25] Van toc dat banh trai A [mm/s] */
    int16_t  va_act_mms;        /* [26..27] Van toc thuc banh trai A [mm/s] */
    int16_t  vb_ref_mms;        /* [28..29] Van toc dat banh phai B [mm/s] */
    int16_t  vb_act_mms;        /* [30..31] Van toc thuc banh phai B [mm/s] */
    int16_t  pwm_a_cpct;        /* [32..33] PWM dong co A [0.01 %] */
    int16_t  pwm_b_cpct;        /* [34..35] PWM dong co B [0.01 %] */
    uint16_t s_enc_mm;          /* [36..37] Quang duong odometry trong chang hien tai [mm] */
    uint16_t total_i_m;         /* [38..39] Tong cuong do phan xa total_I * 1000 */
    uint16_t inner_jitter_us;   /* [40..41] Do lech chu ky vong trong Core 1 [us] */
    uint8_t  norm_s[7];         /* [42..48] 7 mat TCRT5000 chuan hoa 0..100 [%] (S1..S7) */
    uint8_t  state_flags;       /* [49]     mission_state [3:0] + cac co trang thai [7:4] */
    uint8_t  dt_smc_ms;         /* [50]     Chu ky thuc te cua bo lap lich SMC [ms] */
    uint8_t  load_class;        /* [51]     Phan loai tai hien tai (0 / 1 / 2 kg) */
} robot_log_sample_t;

_Static_assert(sizeof(robot_log_sample_t) == 52, "robot_log_sample_t must be exactly 52 bytes");

/* Tom tat luot chay gui trong goi PKT_STATUS cho MATLAB (dung 44 byte) */
typedef struct __attribute__((packed)) {
    uint8_t  run_mode;          /* robot_run_mode_t */
    uint8_t  sensor_profile;    /* 5 hoac 7 */
    uint8_t  stop_reason;       /* robot_stop_reason_t */
    uint8_t  load_class_kg;     /* 0, 1, hoac 2 */
    uint32_t sample_count;      /* Tong so mau dang co trong SRAM */
    uint32_t run_duration_ms;   /* Tong thoi gian chay [ms] */
    float    load_measured_g;   /* Khoi luong do thuc te tu HX711 [g] */
    float    max_abs_e2_mm;     /* |e2_filt| cuc dai (sau 0.6s khoi hanh) [mm] */
    float    rms_e2_mm;         /* Sai so e2_filt RMS [mm] */
    float    station_stop_s_m;  /* Odometry luc dung tai Tram tai hang [m] (chuan 2.000 m) */
    float    goal_stop_s_m;     /* Odometry luc dung tai Dich [m] (chuan 4.7091 m) */
    float    goal_creep_mm;     /* Quang duong bo thang sau EOL dua truc banh vao dich [mm] */
    float    avg_dt_smc_ms;     /* Chu ky trung binh cua vong ngoai SMC thich nghi [ms] */
    uint32_t y_gn_calls;        /* So lan kich hoat tach nhanh Gauss-Newton */
} robot_run_summary_t;

_Static_assert(sizeof(robot_run_summary_t) == 44, "robot_run_summary_t must be exactly 44 bytes");

/* Cap phat bo dem SRAM (chi goi khi bat dau He B, Wi-Fi dang TAT) */
esp_err_t data_logger_prepare(uint32_t max_samples);

/* Xoa noi dung va giai phong toan bo SRAM (sau khi burst xong hoac huy) */
void      data_logger_free(void);

/* Ghi them 1 mau vao SRAM (O(1), khong cap phat them trong vong lap thoi gian thuc) */
bool      data_logger_push(const robot_log_sample_t *sample);

/* Doc mau thu `index` (dung khi burst ra UDP) */
bool      data_logger_get(uint32_t index, robot_log_sample_t *out);

uint32_t  data_logger_count(void);
uint32_t  data_logger_capacity(void);
bool      data_logger_is_full(void);

/* Luu / doc tom tat luot chay gan nhat */
void      data_logger_set_summary(const robot_run_summary_t *summary);
void      data_logger_get_summary(robot_run_summary_t *out);

#ifdef __cplusplus
}
#endif
