#pragma once

/* ---------------- Motor Driver (DRV8871 + 6N137) ---------------- */
#define PIN_MOTOR_A1        12      /* PWMA1 (strapping pin, LOW at boot) */
#define PIN_MOTOR_A2        13      /* PWMA2 */
#define PIN_MOTOR_B1        16      /* PWMB1 */
#define PIN_MOTOR_B2        15      /* PWMB2 (strapping pin) */

#define MOTOR_PWM_FREQ_HZ   10000   /* 10 kHz */
#define MOTOR_PWM_RES_BITS  12      /* 12-bit: 0..4095 */
#define MOTOR_PWM_MAX_DUTY  ((1U << MOTOR_PWM_RES_BITS) - 1U)

/* ---------------- Encoders (Quadrature 4x) ---------------- */
#define PIN_ENC_A_A         21      /* ENCA-A */
#define PIN_ENC_A_B         19      /* ENCA-B */
#define PIN_ENC_B_A         18      /* ENCB-A */
#define PIN_ENC_B_B         17      /* ENCB-B */

/* Số xung 4x mỗi vòng trục bánh xe (Sửa lại theo động cơ thực tế)
 * Ví dụ: encoder 11 PPR, hộp số 1:30 -> 11 * 30 * 4 = 1320.0f */
#define ENC_CPR_A           1320.0f
#define ENC_CPR_B           1320.0f

/* Đảo dấu (+1 hoặc -1) nếu chiều quay dương PWM cho RPM âm */
#define ENC_DIR_A           (+1)
#define ENC_DIR_B           (+1)

/* ---------------- User I/O ---------------- */
#define PIN_USER_LED        2       /* USR_LED trên PCB */
#define PIN_BOOT_BUTTON     0       /* BOOT button (active LOW) */

/* ---------------- OTA ---------------- */
#define OTA_WINDOW_MS       30000           /* 15 giây chờ nạp OTA sau reset */
#define OTA_WIFI_SSID       "LineRobot-OTA"
#define OTA_WIFI_PASS       "12345678"

/* ---------------- SysID Wi-Fi & HTTP Server ---------------- */
#define SYSID_WIFI_SSID     "LineRobot-SYSID"
#define SYSID_WIFI_PASS     "12345678"
#define SYSID_HTTP_PORT     8080

/* ---------------- SysID Sampling & HTTP Server ---------------- */
#define SYSID_FS_HZ         200             /* Tần số lấy mẫu 200 Hz */
#define SYSID_TS_US         5000            /* Chu kỳ 5000 us */
#define SYSID_MAX_SAMPLES   6000            /* 30 s * 200 Hz = 6000 mẫu (72 KB) */