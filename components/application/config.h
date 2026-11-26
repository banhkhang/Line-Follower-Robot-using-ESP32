#pragma once
#include "driver/gpio.h"

<<<<<<< HEAD
/* ------------------------------------------------------------------ */
/* Hardware pins (YD-ESP32-S3 N16R8)                                  */
/* ------------------------------------------------------------------ */
#define PIN_BOOT_BUTTON        0        /* BOOT button, pressed = LOW */
=======
/*
 * config.h - the ONE place for pins, peripheral settings, and wireless config.
 *
 * If you change a wire on the PCB, change it here and nowhere else.
 * Pin names follow the schematic (ESP32-WROOM custom board).
 *
 * Note: Kinematic, dynamic, motor plant, and 2-DOF PI gain-scheduling constants
 * live in `robot_params.h` to keep hardware pinout cleanly separated from math.
 */
>>>>>>> b3bd95d5874f98f3172a4e590f05b1fae8eedb51

/* ------------------------------------------------------------------ */
/* Wi-Fi SoftAP (shared between OTA boot window & HIL UDP)            */
/* Default ESP-IDF SoftAP IP: 192.168.4.1, PC client IP: 192.168.4.2  */
/* ------------------------------------------------------------------ */
<<<<<<< HEAD
#define HIL_WIFI_SSID          "ESP32S3_HIL_BOT"
#define HIL_WIFI_PASS          "12345678"

#define OTA_WIFI_SSID          HIL_WIFI_SSID
#define OTA_WIFI_PASS          HIL_WIFI_PASS
#define OTA_WINDOW_MS          20000    /* 20 s; nhấn BOOT để bỏ qua ngay */
#define KEEP_WIFI_AP_AFTER_OTA 1        /* 1 = giữ SoftAP chạy tiếp cho HIL UDP */
=======
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
>>>>>>> b3bd95d5874f98f3172a4e590f05b1fae8eedb51

/* ------------------------------------------------------------------ */
/* HIL UDP protocol (matches Simulink R2024a model)                   */
/* ------------------------------------------------------------------ */
#define ESP_UDP_PORT           8889     /* Local port ESP32 (Simulink Remote) */
#define PC_UDP_PORT            8888     /* PC port Simulink (UDP Receive)     */
#define HIL_RX_PACKET_LEN      21       /* Simulink -> ESP32 (0xAA)           */
#define HIL_TX_PACKET_LEN      6        /* ESP32 -> Simulink (0xBB)           */
#define HIL_LOCKSTEP_SYNC      1        /* 1 = chạy 1 bước điều khiển ngay khi nhận gói UDP */
#define HIL_FAILSAFE_MS        1000     /* Reset trạng thái khi Simulink dừng > 1.0 s */

<<<<<<< HEAD
/* ------------------------------------------------------------------ */
/* Mechanical & Sensor Geometry                                       */
/* ------------------------------------------------------------------ */
#define TS_SEC                 0.005f   /* 200 Hz (5000 us)                   */
#define WHEEL_RADIUS_M         0.0325f  /* R = 32.5 mm                        */
#define TRACK_WIDTH_M          0.190f   /* B = 190 mm                         */
#define M_CHASSIS_KG           2.0f     /* Khối lượng xe rỗng                 */

/* 0.140f cho Simulink read_sensors; đổi sang 0.075f khi chạy xe thật */
#define SENSOR_LOOKAHEAD_M     0.075f

/* Crossbar & Loading Station */
#define CROSSBAR_SENSOR_TH      0.35f
#define CROSSBAR_MIN_ACTIVE     5
#define CROSSBAR_DEBOUNCE_SEC   0.020f   /* 4 chu kỳ (20 ms / 52 ms bề rộng vạch) */
#define LOAD_APPROACH_DIST_M    0.160f   /* Tiến thêm 160 mm + 47 mm phanh => dừng tại X ~ 1008 mm */
#define LOAD_STOP_DWELL_SEC     2.0f     /* Chờ nhận tải 2.0 s trong vùng [850, 1150] */
=======
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
>>>>>>> b3bd95d5874f98f3172a4e590f05b1fae8eedb51

/* ------------------------------------------------------------------ */
/* HIL Run Mode Selection                                             */
/* ------------------------------------------------------------------ */
#define MODE_LINE_FOLLOW       0   /* Chế độ bám line đầy đủ */
#define MODE_INNER_STEP_TEST   1   /* Bước 1: Chỉ test vòng trong (S-curve + bậc thang) */

<<<<<<< HEAD
#define HIL_RUN_MODE           MODE_LINE_FOLLOW

/* Outer-Loop Gains (Step 2 - tăng độ cứng bám chuyển tiếp cung, Ls = 0.075 m) */
#define V_CRUISE_MAX            0.50f
#define V_CRUISE_MIN            0.30f
#define V_WHEEL_MIN             0.00f
#define V_WHEEL_MAX             0.75f
#define OMEGA_MAX_RADS          4.50f
#define LONGITUDINAL_ACCEL_MAX  1.80f    /* Chỉ giới hạn gia tốc dài của xe, không bóp lực lái */

#define K1_KINEMATIC            4.50f    /* Tăng từ 2.50 -> 4.50 */
#define K_SMC_GAIN              0.075f   /* Tăng từ 0.045 -> 0.075 */
#define PHI_BL_M                0.012f   /* Thu hẹp lớp biên 0.015 -> 0.012 (K_tot = 10.75 s^-1) */
#define KD_RATE_SEC             0.030f   /* Tăng tắt dần đạo hàm */
#define KI_SMC_GAIN             2.50f
#define Z_INT_MAX_M             0.006f

/* Băng thông thiết kế vòng trong [rad/s] (chỉnh duy nhất số này khi tune PI) */
#define INNER_LOOP_WGC         30.0f

/* ------------------------------------------------------------------ */
/* Two-Stage Goal Stopping (X = 3000 mm, no goal crossbar)            */
/* ------------------------------------------------------------------ */
#define GOAL_ARM_DIST_M         4.400f   /* Mở khóa EOL trên đoạn thẳng cuối (X ~ 2500 mm) */
#define GOAL_DECEL_START_M      4.600f   /* Bắt đầu giảm tốc về v_creep (X ~ 2701 mm)      */
#define GOAL_CREEP_SPEED_MS     0.08f    /* Tốc độ bò tiếp cận mép hết line [m/s]          */
#define GOAL_DECEL_RATE_MS2     1.00f    /* Gia tốc giảm tốc tầng 1 [m/s^2]                */
#define GOAL_STEER_FREEZE_I     1.20f    /* Khóa lái thẳng khi total_I sụt qua mép line    */

#define GOAL_EOL_DEBOUNCE_SEC   0.010f   /* 2 chu kỳ (10 ms) xác nhận hết line cả 7 mắt    */
#define GOAL_EOL_OFFSET_M       0.0708f  /* 75.0 mm (X_sens) - 4.2 mm (PSF + 10 ms deb)    */
#define GOAL_FINAL_DECEL_MS2    0.12f    /* Độ dốc quỹ đạo sqrt(2*a*d_rem) tầng 2 [m/s^2]  */
#define GOAL_MIN_CREEP_MS       0.025f   /* Tốc độ bò tối thiểu trước khi đóng Short Brake */
#define GOAL_STOP_BAND_M        0.0006f  /* Đóng Short Brake khi còn <= 0.6 mm             */
=======
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
>>>>>>> b3bd95d5874f98f3172a4e590f05b1fae8eedb51
