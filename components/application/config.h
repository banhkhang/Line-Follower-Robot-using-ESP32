#pragma once
#include "driver/gpio.h"

/*
 * config.h - the ONE place for pins, peripheral settings, and wireless config.
 *
 * If you change a wire on the PCB, change it here and nowhere else.
 * Pin names follow the schematic (ESP32-WROOM custom board).
 *
 * Note: Kinematic, dynamic, motor plant, and 2-DOF PI gain-scheduling constants
 * live in `robot_params.h` to keep hardware pinout cleanly separated from math.
 */

/* ------------------------------------------------------------------ */
/* Load cell (HX711)                                                   */
/* ------------------------------------------------------------------ */
#define PIN_HX711_DOUT   14   /* schematic net "DOUT"   */
#define PIN_HX711_SCK    27   /* schematic net "PD_SCK" */
#define HX711_GAIN       128    /* 128 (25 pulses) or 64 (27 pulses) on Channel A */

#define LOADCELL_SAMPLES     5      /* readings averaged per update (HX711 = 10 per second) */
#define LOADCELL_TIMEOUT_MS  1000   /* give up if the HX711 does not answer in this time    */

/* Load cell rated capacity */
#define HX711_LOADCELL_CAPACITY_G   5000.0f
/* Number of samples used for tare */
#define HX711_TARE_SAMPLES          10
/* Calibration factor: to be determined experimentally */
#define HX711_CALIBRATION_FACTOR    1.0f

/* ------------------------------------------------------------------ */
/* Line sensor array (7x TCRT5000), analog inputs                      */
/* ------------------------------------------------------------------ */
#define PIN_SENSOR_1     36   /* A1 - ADC1_CH0 (input only) */
#define PIN_SENSOR_2     39   /* A2 - ADC1_CH3 (input only) */
#define PIN_SENSOR_3     34   /* A3 - ADC1_CH6 (input only) */
#define PIN_SENSOR_4     35   /* A4 - ADC1_CH7 (input only) */
#define PIN_SENSOR_5     32   /* A5 - ADC1_CH4 */
#define PIN_SENSOR_6     33   /* A6 - ADC1_CH5 */
#define PIN_SENSOR_7     25   /* A7 - ADC2_CH8 (ADC2 cannot be used while Wi-Fi is on) */
#define PIN_SENSOR_CTRL  26   /* schematic net "CTRL" - assumed sensor emitter enable, please verify */

#define LINE_SENSOR_MAX_COUNT  7

/* 5 Sensors configuration variables */
#define EDGE_CORR_START_MM   7.44f
#define EDGE_CORR_GAIN       1.5132f

/* ------------------------------------------------------------------ */
/* Motor driver (DRV8871 + 6N137 optocouplers) & Hall encoders        */
/* ------------------------------------------------------------------ */
#define PIN_MOTOR_A1     12   /* PWMA1 (Motor A / LEFT) - strapping pin, must be LOW at boot */
#define PIN_MOTOR_A2     13   /* PWMA2 (Motor A / LEFT) */
#define PIN_MOTOR_B1     16   /* PWMB1 (Motor B / RIGHT) */
#define PIN_MOTOR_B2     15   /* PWMB2 (Motor B / RIGHT) - strapping pin */

/* LEDC PWM: 10 kHz, 12-bit (4096 levels -> ~0.30 mm/s quantization step) */
#define MOTOR_PWM_FREQ_HZ   10000
#define MOTOR_PWM_RES_BITS  12
#define MOTOR_PWM_MAX_DUTY  ((1U << MOTOR_PWM_RES_BITS) - 1U)

/* Hall encoders (M/T 1x mode on Core 1; channel B pins MUST be < 32 for GPIO_IN_REG) */
#define PIN_ENC_A_A      21   /* ENCA-A (Motor A / LEFT, rising-edge ISR) */
#define PIN_ENC_A_B      19   /* ENCA-B (Motor A / LEFT, direction logic) */
#define PIN_ENC_B_A      18   /* ENCB-A (Motor B / RIGHT, rising-edge ISR) */
#define PIN_ENC_B_B      17   /* ENCB-B (Motor B / RIGHT, direction logic) */

/* Invert Motor A encoder sign so both wheels read positive when driving forward */
#define ENC_DIR_A        (-1)
#define ENC_DIR_B        (+1)

/* ------------------------------------------------------------------ */
/* User I/O                                                            */
/* ------------------------------------------------------------------ */
#define PIN_USER_LED      2   /* USR_LED (strapping pin) */
//#define PIN_USER_BUTTON   4   /* USR_BTN, external pull-up, pressed = LOW */

/* ---------------- OTA (wireless update - stable main config) ---------------- */
#define PIN_BOOT_BUTTON   0        /* BOOT button, pressed = LOW (press it AFTER reset, not during) */
#define OTA_WINDOW_MS     100000   /* how long to wait for an upload after reset */
#define OTA_WIFI_SSID     "LineRobot-OTA"
#define OTA_WIFI_PASS     "12345678"   /* at least 8 characters */

/* ---------------- Telemetry (wireless log) ---------------- */
#define USE_TELEMETRY         0             /* 0 = no Wi-Fi at all while the robot runs */
#define TELEMETRY_WIFI_SSID   "LineRobot-LOG"
#define TELEMETRY_WIFI_PASS   "12345678"
#define TELEMETRY_PORT        5000
#define TELEMETRY_DEST_IP     "192.168.4.255"   /* broadcast: any PC on this Wi-Fi hears it */
#define TELEMETRY_ALSO_UART   1                 /* 1 = also print on the USB cable (debugging) */

#if USE_TELEMETRY
#define TCRT5000_ACTIVE_CHANNELS 5   /* Symmetric center window: S2, S3, S4, S5, S6 */
#else
#define TCRT5000_ACTIVE_CHANNELS 7   /* Full array: S1..S7 */
#endif
