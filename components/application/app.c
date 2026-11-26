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
#include "wifi_ap.h"
#include "telemetry.h"
#include "loadcell_service.h"
#include "drive_service.h"
#include "robot_params.h"

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

void app_start(void)
{
    /* 0. AN TOAN PHAN CUNG: Khoa cau H o trang thai Coast (Hi-Z) NGAY KHI BOOT
     *    de dam bao dong co khong giat/quay trong suot cua so OTA. */
    drive_service_lock_motors();

    esp_reset_reason_t reason = esp_reset_reason();

    ESP_LOGW("BOOT", "================================");
    ESP_LOGW("BOOT", "ESP32 RESTART DETECTED");
    ESP_LOGW("BOOT", "Reset reason: %d", reason);
    ESP_LOGW("BOOT", "================================");

    /* 1. Start log capture immediately (queues logs until Wi-Fi is up) */
    wifi_ap_init_base();          /* Start lwIP + event loop once */
    telemetry_init();

    ESP_LOGI(TAG, "Starting up");

    /* 2. Open OTA window (turns on LineRobot-OTA; boot logs broadcast here!) */
    ota_boot_window();

    /* 3. Khoi chay vong dieu khien van toc ben trong (Core 1, 200 Hz) sau OTA
     *    de tranh tranh chap ngat 200 Hz trong luc OTA ghi Flash. */
    if (drive_service_start() != ESP_OK) {
        ESP_LOGE(TAG, "Inner velocity loop (drive_service) start failed");
        return;
    }
    drive_service_set_payload_kg(0.0f);
    drive_service_reset_odometry();

    /* 4. Khoi tao & Tare Loadcell HX711 (chay tren Core 0, khong anh huong Core 1) */
    bool loadcell_ok = (loadcell_service_init() == ESP_OK);
    if (!loadcell_ok) {
        ESP_LOGW(TAG, "Load cell init failed - tiep tuc chay voi mac dinh m_load = 0 kg");
    } else {
        /* Zero the scale. The load cell must be empty right now. */
        ESP_LOGI(TAG, "Taring in 2 s - keep the load cell EMPTY");
        vTaskDelay(pdMS_TO_TICKS(2000));

        if (loadcell_service_tare() != ESP_OK) {
            ESP_LOGW(TAG, "No answer from HX711 - check DOUT/SCK wiring and 3V3 power");
            loadcell_ok = false;
        } else {
            ESP_LOGI(TAG, "Tare done, offset = %ld", (long)loadcell_service_get_offset());
        }
    }

    /*
     * 5. Vong lap chinh (Core 0):
     *    Hien tai doc Loadcell & giam sat trang thai vong trong qua Telemetry.
     *    Khi tich hop vong ngoai (7x TCRT5000 + Luenberger 100 Hz + SMC 20 Hz):
     *      - Dang ky dong bo pha: drive_service_register_outer_notify(outer_task, OBSERVER_LOOP_DIVIDER);
     *      - Doc odometry       : drive_service_get_state(&drv); -> drv.v_mps, drv.w_rads, drv.s_m
     *      - Tai tram tai hang  : dung xe bang drive_service_stop_brake(), doi drive_service_is_stopped(),
     *                             doc HX711 (sau khi hieu chuan LOADCELL_SCALE trong config.h) va goi:
     *                             drive_service_set_payload_kg(r.grams * 0.001f);
     *                             drive_service_reset_odometry();
     *      - Khoi hanh muot     : drive_service_start_scurve(ROBOT_V_NOM_MPS, ROBOT_SCURVE_DEFAULT_T_S);
     *      - Bam line SMC       : drive_service_set_chassis_vel(v_cmd, w_cmd);
     */
    while (1) {
        drive_state_t drv;
        drive_service_get_state(&drv);

        if (loadcell_ok) {
            loadcell_reading_t r;
            if (loadcell_service_read(&r) == ESP_OK) {
                ESP_LOGI(TAG,
                         "HX711 raw=%ld net=%ld g=%.2f | DRV v=%.3f w=%.2f s=%.3f m vA=%.3f vB=%.3f",
                         (long)r.raw, (long)r.net, r.grams,
                         drv.v_mps, drv.w_rads, drv.s_m, drv.v_a_mps, drv.v_b_mps);
            } else {
                ESP_LOGW(TAG, "Load cell read failed");
            }
        } else {
            ESP_LOGI(TAG,
                     "DRV mode=%d v=%.3f m/s w=%.2f rad/s s=%.3f m | vA=%.3f vB=%.3f | jitter=%u us",
                     (int)drv.mode, drv.v_mps, drv.w_rads, drv.s_m,
                     drv.v_a_mps, drv.v_b_mps, (unsigned)drv.dt_us_err);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
