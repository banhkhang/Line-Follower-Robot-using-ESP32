#pragma once
#include "driver/gpio.h"

/*
 * ============================================================================
 * config.h - Hardware Pinout, Peripheral Settings & Standalone Runtime Config
 * ============================================================================
 *
 * Single place for hardware pins and system-level configuration matching the
 * custom ESP32-WROOM-32 PCB schematic (Standalone Offline Build: No Wi-Fi).
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
#define PIN_SENSOR_7                25      /* A7 - ADC2_CH8 (Rightmost, +27 mm; free of Wi-Fi!) */
#define PIN_SENSOR_CTRL             26      /* schematic net "CTRL" - IR emitter MOSFET gate */

#define LINE_SENSOR_MAX_COUNT       7
#define TCRT5000_OVERSAMPLE_COUNT   2       /* 2x back-to-back ADC average per ON/OFF phase */

/*
 * Active line-tracking sensor profile (No Wi-Fi -> ADC2 is 100% free):
 *  - TCRT_PROFILE_7S (Default): Full 7 sensors (S1..S7, +-27 mm FOV). Outperformed 5S
 *    on final acceptance runs (Leg 1 RMS 0.85-1.07 mm vs 1.00-1.19 mm, tighter Leg 2 max)
 *    with +50% wider recovery margin.
 *  - Change to TCRT_PROFILE_5S if you want 5 center sensors (S2..S6, ADC1 only).
 */
#define ROBOT_RUN_SENSOR_PROFILE    TCRT_PROFILE_7S

/* 5-sensor (S2..S6) piecewise edge linearization calibrated from 7S vs 5S sweep */
#define EDGE_CORR_START_MM          7.44f
#define EDGE_CORR_GAIN              1.5132f

/* -------------------------------------------------------------------------- */
/* 3. Motor driver (DRV8871 + 6N137 optocouplers) & Hall encoders             */
/* -------------------------------------------------------------------------- */
#define PIN_MOTOR_A1                12      /* PWMA1 (Motor A / RIGHT, S7 side) - strapping pin */
#define PIN_MOTOR_A2                13      /* PWMA2 (Motor A / RIGHT, S7 side) */
#define PIN_MOTOR_B1                16      /* PWMB1 (Motor B / LEFT,  S1 side) */
#define PIN_MOTOR_B2                15      /* PWMB2 (Motor B / LEFT,  S1 side) - strapping pin */

/* LEDC PWM: 10 kHz, 12-bit (4096 levels -> ~0.30 mm/s quantization step) */
#define MOTOR_PWM_FREQ_HZ           10000
#define MOTOR_PWM_RES_BITS          12
#define MOTOR_PWM_MAX_DUTY          ((1U << MOTOR_PWM_RES_BITS) - 1U)

/* Hall encoders (M/T 1x mode on Core 1; channel B pins MUST be < 32 for GPIO_IN_REG) */
#define PIN_ENC_A_A                 21      /* ENCA-A (Motor A / RIGHT, rising-edge ISR) */
#define PIN_ENC_A_B                 19      /* ENCA-B (Motor A / RIGHT, direction logic) */
#define PIN_ENC_B_A                 18      /* ENCB-A (Motor B / LEFT,  rising-edge ISR) */
#define PIN_ENC_B_B                 17      /* ENCB-B (Motor B / LEFT,  direction logic) */

/* Invert Motor A encoder sign so both wheels read positive when driving forward */
#define ENC_DIR_A                   (-1)
#define ENC_DIR_B                   (+1)

/* -------------------------------------------------------------------------- */
/* 4. User I/O (ESP32-WROOM PCB: LED1 on GPIO2, BOOT on GPIO0, EN = Reset)    */
/* -------------------------------------------------------------------------- */
#define PIN_USER_LED                2       /* USR_LED / LED1 (active HIGH) */
#define PIN_BOOT_BUTTON             0       /* BOOT button (active LOW, pressed AFTER reset) */

/* BOOT button gesture timing in IDLE mode:
 *  - Short press & release -> Start line-tracking + load-carrying mission (3 s countdown)
 *  - Hold >= 2.0 s         -> Enter 7-sensor NVS sweep calibration */
#define BTN_DEBOUNCE_MS             35U     /* Debounce window [ms] */
#define BTN_LONG_HOLD_MS            2000U   /* Hold >= 2.0 s to enter 7-sensor NVS calibration */

/* LED1 prelude before entering 7-sensor calibration:
 * ON 1s, OFF 1s, repeated 3 times; then SOLID ON during sweep; OFF when finished. */
#define CALIB_PRELUDE_CYCLES        3
#define CALIB_PRELUDE_ON_MS         1000U
#define CALIB_PRELUDE_OFF_MS        1000U

/* Pre-run countdown [ms] after pressing BOOT once */
#define RUN_COUNTDOWN_MIN_MS        3000U
