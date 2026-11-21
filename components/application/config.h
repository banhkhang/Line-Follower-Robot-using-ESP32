#pragma once

/* ---------------- Motor Driver (DRV8871 + 6N137) ---------------- */
#define PIN_MOTOR_A1        12
#define PIN_MOTOR_A2        13
#define PIN_MOTOR_B1        16
#define PIN_MOTOR_B2        15

#define MOTOR_PWM_FREQ_HZ   10000
#define MOTOR_PWM_RES_BITS  12
#define MOTOR_PWM_MAX_DUTY  ((1U << MOTOR_PWM_RES_BITS) - 1U)

/* ---------------- Encoders & Wheel Geometry ---------------- */
#define PIN_ENC_A_A         21
#define PIN_ENC_A_B         19
#define PIN_ENC_B_A         18
#define PIN_ENC_B_B         17

/* JGB37-520: 11 PPR * 30:1 * 4x = 1320 xung/vòng trục ra */
#define ENC_CPR_A           1320.0f
#define ENC_CPR_B           1320.0f

/* Đã đảo dấu Motor A để cả 2 kênh cùng dương khi PWM dương */
#define ENC_DIR_A           (-1)
#define ENC_DIR_B           (+1)

/* Bánh xe V2 D = 65 mm -> C = pi * 0.065 = 0.2042035 m */
#define WHEEL_DIAMETER_M    0.065f
#define WHEEL_CIRCUM_M      0.20420352f
#define RPM_TO_MPS          (WHEEL_CIRCUM_M / 60.0f)

/* ---------------- User I/O ---------------- */
#define PIN_USER_LED        2
#define PIN_BOOT_BUTTON     0

/* ---------------- OTA (giữ nguyên cấu trúc flash Wi-Fi) ---------------- */
#define OTA_WINDOW_MS       30000
#define OTA_WIFI_SSID       "LineRobot-OTA"
#define OTA_WIFI_PASS       "12345678"

/* ---------------- SysID / Closed-Loop Wi-Fi & Sampling ---------------- */
#define SYSID_WIFI_SSID     "LineRobot-SYSID"
#define SYSID_WIFI_PASS     "12345678"
#define SYSID_HTTP_PORT     8080

#define SYSID_FS_HZ         200
#define SYSID_TS_US         5000
#define SYSID_TS_S          0.005f
#define LOG_MAX_SAMPLES     6000    /* Đủ cho cả id30 (6000) và cl10 (2000) */