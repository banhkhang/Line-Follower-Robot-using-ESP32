#pragma once

/*
 * config.h - the ONE place for pins and tunable numbers.
 *
 * If you change a wire on the PCB, change it here and nowhere else.
 * Pin names follow the schematic (ESP32-WROOM custom board).
 */

/* ------------------------------------------------------------------ */
/* Load cell (HX711)                                                   */
/* ------------------------------------------------------------------ */
#define PIN_HX711_DOUT   14   /* schematic net "DOUT"   */
#define PIN_HX711_SCK    27   /* schematic net "PD_SCK" */

#define LOADCELL_SAMPLES     5      /* readings averaged per update (HX711 = 10 per second) */
#define LOADCELL_TIMEOUT_MS  1000   /* give up if the HX711 does not answer in this time    */

/*
 * Counts per gram. Leave at 1.0 until you calibrate (see README).
 * While it is 1.0 the "grams" printed on the serial monitor are just raw counts.
 */
#define LOADCELL_SCALE   1.0f

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

/* ------------------------------------------------------------------ */
/* Motor driver (2 channels, 2 PWM inputs each) and encoders           */
/* ------------------------------------------------------------------ */
#define PIN_MOTOR_A1     12   /* PWMA1 - strapping pin, must be LOW at boot */
#define PIN_MOTOR_A2     13   /* PWMA2 */
#define PIN_MOTOR_B1     16   /* PWMB1 */
#define PIN_MOTOR_B2     15   /* PWMB2 - strapping pin */

#define PIN_ENC_A_A      21   /* ENCA-A */
#define PIN_ENC_A_B      19   /* ENCA-B */
#define PIN_ENC_B_A      18   /* ENCB-A */
#define PIN_ENC_B_B      17   /* ENCB-B */

/* ------------------------------------------------------------------ */
/* User I/O                                                            */
/* ------------------------------------------------------------------ */
#define PIN_USER_LED      2   /* USR_LED (strapping pin) */
//#define PIN_USER_BUTTON   4   /* USR_BTN, external pull-up, pressed = LOW */

/* ---------------- OTA (wireless update) ---------------- */
#define PIN_BOOT_BUTTON   0        /* BOOT button, pressed = LOW (press it AFTER reset, not during) */
#define OTA_WINDOW_MS     100000    /* how long to wait for an upload after reset */
#define OTA_WIFI_SSID     "LineRobot-OTA"
#define OTA_WIFI_PASS     "12345678"   /* at least 8 characters */

/* ---------------- Telemetry (wireless log) ---------------- */
#define USE_TELEMETRY         0             /* 0 = no Wi-Fi at all while the robot runs */
#define TELEMETRY_WIFI_SSID   "LineRobot-LOG"
#define TELEMETRY_WIFI_PASS   "12345678"
#define TELEMETRY_PORT        5000
#define TELEMETRY_DEST_IP     "192.168.4.255"   /* broadcast: any PC on this Wi-Fi hears it */
#define TELEMETRY_ALSO_UART   0                 /* 1 = also print on the USB cable (debugging) */