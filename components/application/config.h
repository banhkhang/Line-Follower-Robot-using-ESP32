#pragma once

/* ------------------------------------------------------------------ */
/* Hardware pins (YD-ESP32-S3 N16R8)                                  */
/* ------------------------------------------------------------------ */
#define PIN_BOOT_BUTTON        0        /* BOOT button, pressed = LOW */

/* ------------------------------------------------------------------ */
/* Wi-Fi SoftAP (shared between OTA boot window & HIL UDP)            */
/* Default ESP-IDF SoftAP IP: 192.168.4.1, PC client IP: 192.168.4.2  */
/* ------------------------------------------------------------------ */
#define HIL_WIFI_SSID          "ESP32S3_HIL_BOT"
#define HIL_WIFI_PASS          "12345678"

#define OTA_WIFI_SSID          HIL_WIFI_SSID
#define OTA_WIFI_PASS          HIL_WIFI_PASS
#define OTA_WINDOW_MS          20000    /* 20 s; nhấn BOOT để bỏ qua ngay */
#define KEEP_WIFI_AP_AFTER_OTA 1        /* 1 = giữ SoftAP chạy tiếp cho HIL UDP */

/* ------------------------------------------------------------------ */
/* HIL UDP protocol (matches Simulink R2024a model)                   */
/* ------------------------------------------------------------------ */
#define ESP_UDP_PORT           8889     /* Local port ESP32 (Simulink Remote) */
#define PC_UDP_PORT            8888     /* PC port Simulink (UDP Receive)     */
#define HIL_RX_PACKET_LEN      21       /* Simulink -> ESP32 (0xAA)           */
#define HIL_TX_PACKET_LEN      6        /* ESP32 -> Simulink (0xBB)           */
#define HIL_LOCKSTEP_SYNC      1        /* 1 = chạy 1 bước điều khiển ngay khi nhận gói UDP */
#define HIL_FAILSAFE_MS        1000     /* Reset trạng thái khi Simulink dừng > 1.0 s */

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

/* ------------------------------------------------------------------ */
/* HIL Run Mode Selection                                             */
/* ------------------------------------------------------------------ */
#define MODE_LINE_FOLLOW       0   /* Chế độ bám line đầy đủ */
#define MODE_INNER_STEP_TEST   1   /* Bước 1: Chỉ test vòng trong (S-curve + bậc thang) */

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