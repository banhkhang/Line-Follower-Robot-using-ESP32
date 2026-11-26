#pragma once
#include "driver/gpio.h"

/*
 * ============================================================================
 * config.h - Hardware Pinout, Peripheral Settings, Wireless & Runtime Config
 * ============================================================================
 *
 * Single place for hardware pins and system-level configuration matching the
 * custom ESP32-WROOM-32 PCB schematic.
 * Kinematic, dynamic, motor plant, 2-DOF PI, and outer-loop SMC math constants
 * live in `robot_params.h`.
 */

/* -------------------------------------------------------------------------- */
/* 1. Load cell (HX711)                                                       */
/* -------------------------------------------------------------------------- */
#define PIN_HX711_DOUT              14      /* schematic net "DOUT"   */
#define PIN_HX711_SCK               27      /* schematic net "PD_SCK" */
#define HX711_GAIN                  128     /* 128 (25 pulses) or 64 (27 pulses) on Channel A */

#define LOADCELL_SAMPLES            5       /* readings averaged per blocking update (10 SPS) */
#define LOADCELL_TIMEOUT_MS         1000    /* timeout if HX711 does not answer */
#define HX711_LOADCELL_CAPACITY_G   5000.0f
#define HX711_TARE_SAMPLES          10

/*
 * Experimental calibration factor (counts / gram): 389.0 or -388.7 depending on A+/A- polarity.
 * With HX711_AUTO_ABS_WEIGHT = 1, grams = |net / factor| so swapped A+/A- wires still read positive!
 */
#define HX711_CALIBRATION_FACTOR    388.7f
#define HX711_AUTO_ABS_WEIGHT       1

/* -------------------------------------------------------------------------- */
/* 2. Line sensor array (7x TCRT5000), analog inputs + low-side IR MOSFET     */
/* -------------------------------------------------------------------------- */
#define PIN_SENSOR_1                36      /* A1 - ADC1_CH0 (Leftmost,  -27 mm on PCB) */
#define PIN_SENSOR_2                39      /* A2 - ADC1_CH3 (-18 mm) */
#define PIN_SENSOR_3                34      /* A3 - ADC1_CH6 ( -9 mm) */
#define PIN_SENSOR_4                35      /* A4 - ADC1_CH7 (  0 mm, Center) */
#define PIN_SENSOR_5                32      /* A5 - ADC1_CH4 ( +9 mm) */
#define PIN_SENSOR_6                33      /* A6 - ADC1_CH5 (+18 mm) */
#define PIN_SENSOR_7                25      /* A7 - ADC2_CH8 (Rightmost, +27 mm; conflicts with Wi-Fi!) */
#define PIN_SENSOR_CTRL             26      /* schematic net "CTRL" - IR emitter MOSFET gate */

#define LINE_SENSOR_MAX_COUNT       7
#define TCRT5000_OVERSAMPLE_COUNT   2       /* 2x back-to-back ADC average per ON/OFF phase */

/* 5-sensor (S2..S6) piecewise edge linearization calibrated from 7S vs 5S sweep */
#define EDGE_CORR_START_MM          7.44f
#define EDGE_CORR_GAIN              1.5132f

/* -------------------------------------------------------------------------- */
/* 3. Motor driver (DRV8871 + 6N137 optocouplers) & Hall encoders             */
/* -------------------------------------------------------------------------- */
#define PIN_MOTOR_A1                12      /* PWMA1 (Motor A / LEFT)  - strapping pin */
#define PIN_MOTOR_A2                13      /* PWMA2 (Motor A / LEFT) */
#define PIN_MOTOR_B1                16      /* PWMB1 (Motor B / RIGHT) */
#define PIN_MOTOR_B2                15      /* PWMB2 (Motor B / RIGHT) - strapping pin */

/* LEDC PWM: 10 kHz, 12-bit (4096 levels -> ~0.30 mm/s quantization step) */
#define MOTOR_PWM_FREQ_HZ           10000
#define MOTOR_PWM_RES_BITS          12
#define MOTOR_PWM_MAX_DUTY          ((1U << MOTOR_PWM_RES_BITS) - 1U)

/* Hall encoders (M/T 1x mode on Core 1; channel B pins MUST be < 32 for GPIO_IN_REG) */
#define PIN_ENC_A_A                 21      /* ENCA-A (Motor A / LEFT, rising-edge ISR) */
#define PIN_ENC_A_B                 19      /* ENCA-B (Motor A / LEFT, direction logic) */
#define PIN_ENC_B_A                 18      /* ENCB-A (Motor B / RIGHT, rising-edge ISR) */
#define PIN_ENC_B_B                 17      /* ENCB-B (Motor B / RIGHT, direction logic) */

/* Invert Motor A encoder sign so both wheels read positive when driving forward */
#define ENC_DIR_A                   (-1)
#define ENC_DIR_B                   (+1)

/* -------------------------------------------------------------------------- */
/* 4. User I/O (ESP32-WROOM PCB: LED1 on GPIO2, BOOT on GPIO0, EN = Reset)    */
/* -------------------------------------------------------------------------- */
#define PIN_USER_LED                2       /* USR_LED / LED1 (active HIGH) */
#define PIN_BOOT_BUTTON             0       /* BOOT button (active LOW, pressed AFTER reset) */
/* #define PIN_USER_BUTTON          4       USR_BTN is NOT populated on current PCB */

/* BOOT button gesture timing in IDLE mode */
#define BTN_DEBOUNCE_MS             35U     /* Debounce window [ms] */
#define BTN_DOUBLE_CLICK_MS         450U    /* Max gap between 2 clicks for 5S standalone */
#define BTN_LONG_HOLD_MS            2000U   /* Hold >= 2.0 s to enter 7-sensor NVS calibration */

/* LED1 prelude before turning off Wi-Fi to enter 7-sensor calibration:
 * ON 1s, OFF 1s, repeated 3 times; then SOLID ON during sweep; OFF when finished. */
#define CALIB_PRELUDE_CYCLES        3
#define CALIB_PRELUDE_ON_MS         1000U
#define CALIB_PRELUDE_OFF_MS        1000U

/* Minimum pre-run countdown [ms]: System A = 3s, System B / Standalone 7S = max(3s, t_switch) */
#define RUN_COUNTDOWN_MIN_MS        3000U

/* -------------------------------------------------------------------------- */
/* 5. Wi-Fi SoftAP, OTA & MATLAB Link (System A Live + System B Burst)        */
/* -------------------------------------------------------------------------- */
/* Unified SoftAP so PC/MATLAB stays connected seamlessly after OTA window closes */
#define ROBOT_WIFI_SSID             "LineRobot-AP"
#define ROBOT_WIFI_PASS             "12345678"

#define OTA_WIFI_SSID               ROBOT_WIFI_SSID
#define OTA_WIFI_PASS               ROBOT_WIFI_PASS
#define OTA_WINDOW_MS               15000   /* 15 s OTA window on reset; press BOOT to skip immediately */
#define KEEP_WIFI_AP_AFTER_OTA      1       /* Keep SoftAP ON for Mode 1 (MATLAB ready) */

/* MATLAB UDP bidirectional control, live telemetry (A), and SRAM burst (B) */
#define MATLAB_CMD_PORT             8889    /* ESP32 listens here for MATLAB commands */
#define MATLAB_DATA_PORT            8888    /* ESP32 transmits status / live / burst packets here */
#define SYSTEM_A_LIVE_DIV           1       /* 1 = 100 Hz live UDP stream (2 = 50 Hz) */

/*
 * System B SRAM Binary Logger Capacity (ESP32-WROOM-32E, no PSRAM):
 *  - Record size = 48 bytes -> 2000 samples = 96,000 bytes (~93.75 KB).
 *  - At 100 Hz active driving (paused/decimated during WAIT_LOAD): covers 20.0 s
 *    (full track at 0.50 m/s takes ~15-17 s + 2 s stop dwell), while keeping >= 75 KB
 *    free DRAM headroom so esp_wifi_init()/esp_wifi_start() succeeds reliably for burst!
 */
#define SYSTEM_B_LOG_MAX_SAMPLES    2000U
#define SYSTEM_B_WAIT_LOG_DIV       20U     /* Log at 5 Hz while waiting at loading station */
#define SYSTEM_B_BURST_CHUNK_SAMPLES 20U    /* 20 * 48 B = 960 B payload per UDP burst packet */

/* Legacy text telemetry UDP defaults (kept for optional debug compatibility) */
#define TELEMETRY_WIFI_SSID         ROBOT_WIFI_SSID
#define TELEMETRY_WIFI_PASS         ROBOT_WIFI_PASS
#define TELEMETRY_PORT              5000
#define TELEMETRY_DEST_IP           "192.168.4.255"
#define TELEMETRY_ALSO_UART         1
