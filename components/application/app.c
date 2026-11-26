/*
 * app.c - Lop ung dung chinh cua Line Follower Robot
 *
 * - Giu nguyen phien ban stable cua Wi-Fi AP, Telemetry UDP, OTA boot window
 *   va Loadcell HX711 tu nhanh main.
 * - Tich hop module dieu khien vong trong `drive_service` (Core 1, 200 Hz,
 *   2-DOF PI + M/T 1x + dong hoc vi sai + luat tham so theo tai trong) de
 *   san sang ghep bo dieu khien vong ngoai (Luenberger 100 Hz + SMC 20 Hz).
 */

#include "app.h"
#include "ota.h"
#include "config.h"
#include "wifi_ap.h"
#include "telemetry.h"
#include "loadcell_service.h"
#include "sensor_service.h"
#include "drive_service.h"
#include "robot_params.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

static void system_services_init(void)
{
    /* 0. AN TOAN PHAN CUNG: Khoa cau H o trang thai Coast (Hi-Z) NGAY KHI BOOT
     *    de dam bao dong co khong giat/quay trong suot cua so OTA. */
    drive_service_lock_motors();

    esp_reset_reason_t reason = esp_reset_reason();

    ESP_LOGW("BOOT", "================================");
    ESP_LOGW("BOOT", "ESP32 RESTART DETECTED");
    ESP_LOGW("BOOT", "Reset reason: %d", reason);
    ESP_LOGW("BOOT", "================================");
  
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_lwip", ESP_LOG_WARN);

    /* 1. Start log capture immediately (queues logs until Wi-Fi is up) */
    wifi_ap_init_base();          /* Start lwIP + event loop once */
    /* 2. Now LwIP's tcpip mbox exists—safe to initialize UDP telemetry */
    telemetry_init();
  
    ESP_LOGI(TAG, "System telemetry ready");

    /* Stage 1: OTA Boot Window (press BOOT to skip) */
    ESP_LOGI(TAG, "Starting up");

    /* 2. Open OTA window (turns on LineRobot-OTA; boot logs broadcast here!) */
    ota_boot_window();
}

static esp_err_t sensors_start(void)
{
    /* Stage 2: Line Sensor Init & Interactive Sweep Calibration */
    esp_err_t err = sensor_service_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Running without line sensor service (%s)", esp_err_to_name(err));
    }
    else {
        sensor_service_calibrate_sweep();
    }
  
    ESP_LOGI(TAG, "Taring in 2 s - keep the load cell EMPTY");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Stage 3: Load Cell (HX711) Init & Tare */
    err = loadcell_service_tare();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Tare failed (%s) - check DOUT/SCK wiring and 3V3 power",
                 esp_err_to_name(err));
        loadcell_service_deinit();
        return;
    }

    ESP_LOGI(TAG, "Tare done, offset = %ld", (long)loadcell_service_get_offset());
    return ESP_OK;
}


void app_start(void)
{
    system_services_init();
  
    if (drive_start() != ESP_OK) {
        return;
    }
  
    sensors_start();
  
    /*
     * Vong lap chinh (Core 0):
     *   Hien tai doc Line Sensor, Loadcell & giam sat trang thai vong trong qua Telemetry.
     *   Khi tich hop vong ngoai (7x TCRT5000 + Luenberger 100 Hz + SMC 20 Hz):
     *     - Dang ky dong bo pha: drive_service_register_outer_notify(outer_task, OBSERVER_LOOP_DIVIDER);
     *     - Doc odometry       : drive_service_get_state(&drv); -> drv.v_mps, drv.w_rads, drv.s_m
     *     - Tai tram tai hang  : dung xe bang drive_service_stop_brake(), doi drive_service_is_stopped(),
     *                            doc HX711 (sau khi hieu chuan LOADCELL_SCALE trong config.h) va goi:
     *                            drive_service_set_payload_kg(r.grams * 0.001f);
     *                            drive_service_reset_odometry();
     *     - Khoi hanh muot     : drive_service_start_scurve(ROBOT_V_NOM_MPS, ROBOT_SCURVE_DEFAULT_T_S);
     *     - Bam line SMC       : drive_service_set_chassis_vel(v_cmd, w_cmd);
     */

    uint8_t loadcell_div = 0;

    while (1) {
        /* 1. Line Sensor Log (every 200 ms) */
        if (sensor_service_is_enabled()) {
            line_sensor_reading_t line;
            if (sensor_service_read(&line) == ESP_OK) {
                /* Build quick 6/7-char visual bar: '#' >= 50%, '+' >= 15%, '.' < 15% */
                char bar[LINE_SENSOR_MAX_COUNT + 1] = {0};
                for (int i = 0; i < line.active_count; i++) {
                    if (line.norm[i] >= 50.0f)      bar[i] = '#';
                    else if (line.norm[i] >= 15.0f) bar[i] = '+';
                    else                            bar[i] = '.';
                }
#if USE_TELEMETRY
                ESP_LOGI(TAG, "LINE [%s] | %3.0f %3.0f %3.0f %3.0f %3.0f %3.0f | err=%+6.2f mm | %s",
                         bar,
                         line.norm[0], line.norm[1], line.norm[2],
                         line.norm[3], line.norm[4], line.norm[5],
                         line.position_mm,
                         line.line_detected ? "OK" : "LOST");
#else
                ESP_LOGI(TAG, "LINE [%s] | %3.0f %3.0f %3.0f %3.0f %3.0f %3.0f %3.0f | err=%+6.2f mm | %s",
                         bar,
                         line.norm[0], line.norm[1], line.norm[2],
                         line.norm[3], line.norm[4], line.norm[5], line.norm[6],
                         line.position_mm,
                         line.line_detected ? "OK" : "LOST");
#endif
            } else {
                ESP_LOGW(TAG, "Line sensor read failed");
            }
        }
      
        /* 2. Inner Drive Loop State Log (every 200 ms) */
        drive_state_t drv;
        drive_service_get_state(&drv);
        ESP_LOGI(TAG,
                 "DRV mode=%d v=%.3f m/s w=%.2f rad/s s=%.3f m | vA=%.3f vB=%.3f | jitter=%u us",
                 (int)drv.mode, drv.v_mps, drv.w_rads, drv.s_m,
                 drv.v_a_mps, drv.v_b_mps, (unsigned)drv.dt_us_err);
      
        /* 3. Load Cell Log (once every 10 cycles = every 2.0 s) */
        if (++loadcell_div >= 10) {
            loadcell_div = 0;
            if (loadcell_service_is_enabled()) {
                loadcell_reading_t r;
                if (loadcell_service_read(&r) == ESP_OK) {
                    ESP_LOGI(TAG, "WEIGHT     | raw=%8ld | net=%8ld | %7.2f g",
                             (long)r.raw, (long)r.net, r.grams);
                } else {
                    ESP_LOGW(TAG, "Load cell read failed");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}