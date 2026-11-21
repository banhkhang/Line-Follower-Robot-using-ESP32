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
#define HIL_FAILSAFE_MS        150      /* Ngắt PWM nếu mất gói > 150 ms      */

/* ------------------------------------------------------------------ */
/* Mechanical & Sensor Geometry                                       */
/* ------------------------------------------------------------------ */
#define TS_SEC                 0.005f   /* 200 Hz (5000 us)                   */
#define WHEEL_RADIUS_M         0.0325f  /* R = 32.5 mm                        */
#define TRACK_WIDTH_M          0.190f   /* B = 190 mm                         */
#define M_CHASSIS_KG           2.0f     /* Khối lượng xe rỗng                 */

/* 0.140f cho Simulink read_sensors; đổi sang 0.075f khi chạy xe thật */
#define SENSOR_LOOKAHEAD_M     0.140f