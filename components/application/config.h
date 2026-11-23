#pragma once

/* Motor pins (hold HIGH on boot -> 6N137 inverted -> DRV8871 Coast/OFF) */
#define PIN_MOTOR_A1            12
#define PIN_MOTOR_A2            13
#define PIN_MOTOR_B1            16
#define PIN_MOTOR_B2            15

/* Loadcell (HX711) */
#define PIN_HX711_DOUT          14
#define PIN_HX711_SCK           27
#define HX711_GAIN_PULSES       1       /* 1 = Ch A Gain 128, 2 = Ch B Gain 32, 3 = Ch A Gain 64 */
#define HX711_READY_TIMEOUT_MS  250     /* 10 SPS = 100 ms period; 250 ms catches stalls */

/* Loadcell Service & Calibration Capture */
#define LC_TASK_CORE            1
#define LC_TASK_PRIO            5
#define LC_TASK_STACK           4096
#define LC_FILTER_WIN           9       /* Odd window size for rolling median/MAD */
#define LC_MAX_CAPTURE_SAMPLES  256     /* 256 samples @ 10 SPS = 25.6 s max */
#define LC_DEFAULT_CAP_SAMPLES  50

/* User I/O */
#define PIN_USER_LED            2
#define PIN_BOOT_BUTTON         0

/* OTA (keep as-is for boot/ota.c) */
#define OTA_WINDOW_MS           30000
#define OTA_WIFI_SSID           "LineRobot-OTA"
#define OTA_WIFI_PASS           "12345678"

/* Calibration SoftAP & HTTP Server */
#define CAL_WIFI_SSID           "LineRobot-CAL"
#define CAL_WIFI_PASS           "12345678"
#define CAL_HTTP_PORT           8080

/* Populated after running MATLAB calibration */
#define HX711_CAL_TARE_COUNTS   561595.012f
#define HX711_CAL_COUNTS_PER_G  -388.694311f