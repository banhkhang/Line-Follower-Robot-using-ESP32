/*
 * ============================================================================
 * app.c - Phien ban doc lap gon nhe (KHONG Wi-Fi, KHONG OTA, KHONG MATLAB/Log)
 * ============================================================================
 *
 * CHUC NANG DUY NHAT (2 che do qua nut BOOT - GPIO0):
 *  [Khoi dong / Nhan nut EN (Reset)]:
 *    1) Khoa cau H Coast (Hi-Z) an toan ngay lap tuc.
 *    2) Khoi tao NVS, LED1 va vong trong `drive_service` (Core 1, 200 Hz, 2-DOF PI).
 *    3) Khoi tao cam bien TCRT5000 (ROBOT_RUN_SENSOR_PROFILE = 7S) & nap calib NVS.
 *    4) Khoi tao & Tare Loadcell HX711 (nhay LED1 2 s).
 *    5) Vao trang thai cho IDLE (LED1 tat).
 *
 *  [Khi dang IDLE]:
 *    - Nhan nut BOOT 1 lan (Short press):
 *        Dem nguoc 3 s (LED1 nhay nhanh) -> Chay bam line + can tai HX711
 *        (1 kg nhanh TRAI / 2 kg nhanh PHAI) -> Dung dung vach dich -> Ve IDLE.
 *        (Nhan BOOT trong luc dem nguoc de huy; giu BOOT >= 150 ms khi dang chay de phanh khan cap.)
 *    - Nhan giu nut BOOT >= 2.0 s (Long hold):
 *        Hieu chuan 7 mat TCRT5000 & luu NVS:
 *          * LED1 nhay canh bao: BAT 1 s / TAT 1 s x 3 lan.
 *          * LED1 SANG LIEN TUC trong luc quet day cam bien qua nen trang & vach den.
 *          * Nhan BOOT 1 lan de ket thuc -> luu vao NVS, tat LED1, ve IDLE.
 * ============================================================================
 */

#include "app.h"
#include "config.h"
#include "robot_params.h"
#include "usr_led.h"
#include "drive_service.h"
#include "sensor_service.h"
#include "loadcell_service.h"
#include "mission_ctrl.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

typedef enum {
    BTN_EVT_NONE = 0,
    BTN_EVT_SHORT_PRESS,
    BTN_EVT_LONG_HOLD
} btn_event_t;

typedef struct {
    bool    pressed;
    bool    long_fired;
    int64_t press_start_ms;
} boot_btn_fsm_t;

static inline int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000LL;
}

static void boot_btn_init(boot_btn_fsm_t *fsm)
{
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << PIN_BOOT_BUTTON,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
    fsm->pressed        = false;
    fsm->long_fired     = false;
    fsm->press_start_ms = 0;
}

static btn_event_t boot_btn_poll(boot_btn_fsm_t *fsm)
{
    bool    is_down = (gpio_get_level(PIN_BOOT_BUTTON) == 0);
    int64_t t       = now_ms();

    if (is_down) {
        if (!fsm->pressed) {
            fsm->pressed        = true;
            fsm->long_fired     = false;
            fsm->press_start_ms = t;
        } else if (!fsm->long_fired && (t - fsm->press_start_ms) >= (int64_t)BTN_LONG_HOLD_MS) {
            fsm->long_fired = true;
            return BTN_EVT_LONG_HOLD;
        }
    } else {
        if (fsm->pressed) {
            int64_t dur = t - fsm->press_start_ms;
            fsm->pressed = false;
            if (!fsm->long_fired && dur >= (int64_t)BTN_DEBOUNCE_MS) {
                return BTN_EVT_SHORT_PRESS;
            }
        }
    }
    return BTN_EVT_NONE;
}

/* Quy trinh hieu chuan 7 cam bien & luu NVS */
static void run_calibration_workflow(void)
{
    ESP_LOGI(TAG, "=== ENTERING 7-SENSOR CALIBRATION WORKFLOW ===");

    /* Doi nguoi dung tha nut BOOT sau khi giu 2 s */
    while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* LED1 canh bao truoc khi quet: BAT 1 s / TAT 1 s x 3 lan */
    for (int i = 0; i < CALIB_PRELUDE_CYCLES; i++) {
        ESP_LOGI(TAG, "Calib prelude %d/%d (LED1 ON 1s / OFF 1s)", i + 1, CALIB_PRELUDE_CYCLES);
        usr_led_set(true);
        vTaskDelay(pdMS_TO_TICKS(CALIB_PRELUDE_ON_MS));
        usr_led_set(false);
        vTaskDelay(pdMS_TO_TICKS(CALIB_PRELUDE_OFF_MS));
    }

    /* Quet du 7 mat TCRT5000 (LED1 sang lien tuc -> nhan BOOT de luu NVS & tat LED1) */
    esp_err_t err = sensor_service_calibrate_sweep();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "7-sensor calibration saved to NVS successfully");
    } else {
        ESP_LOGE(TAG, "Calibration failed (%s)", esp_err_to_name(err));
    }

    /* Khoi phuc profile chay da cau hinh (7S hoac 5S) */
    sensor_service_set_profile(ROBOT_RUN_SENSOR_PROFILE);
}

/* Chay nhiem vu bam line & tai hang */
static void run_line_mission(void)
{
    sensor_service_set_profile(ROBOT_RUN_SENSOR_PROFILE);

    mission_run_cfg_t cfg = {
        .sensor_profile   = ROBOT_RUN_SENSOR_PROFILE,
        .countdown_rem_ms = RUN_COUNTDOWN_MIN_MS
    };

    mission_ctrl_run_blocking(&cfg);
}

static void system_init(void)
{
    /* 0. AN TOAN PHAN CUNG: Khoa cau H o trang thai Coast (Hi-Z) NGAY KHI BOOT */
    drive_service_lock_motors();

    /* Xac nhan firmware hop le neu ban nay vua duoc nap lan cuoi qua Wi-Fi OTA */
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
    }

    ESP_LOGW("BOOT", "========================================");
    ESP_LOGW("BOOT", "LineRobot Standalone Boot (No-WiFi, reset=%d)", (int)esp_reset_reason());
    ESP_LOGW("BOOT", "========================================");

    /* 1. Khoi tao NVS de nap & luu thong so hieu chuan TCRT5000 */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    usr_led_init();

    /* 2. Khoi dong vong trong 200 Hz doc quyen tren Core 1 */
    ESP_ERROR_CHECK(drive_service_start());

    /* 3. Khoi dong cam bien do line (khong Wi-Fi -> dung truc tiep 7S khong xung dot ADC2) */
    if (sensor_service_init(ROBOT_RUN_SENSOR_PROFILE) != ESP_OK) {
        ESP_LOGE(TAG, "Line sensor init failed!");
    }

    /* 4. Khoi dong & Tare Loadcell HX711 (nhay LED1 2 s) */
    if (loadcell_service_init() == ESP_OK) {
        ESP_LOGI(TAG, "Taring HX711 in 2 s - keep load cell EMPTY");
        usr_led_start_blink(250);
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_err_t err = loadcell_service_tare();
        usr_led_set(false);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HX711 Tare OK (offset = %ld, scale = %.1f)",
                     (long)loadcell_service_get_offset(), HX711_CALIBRATION_FACTOR);
        } else {
            ESP_LOGW(TAG, "HX711 Tare failed (%s) -> disabling loadcell", esp_err_to_name(err));
            loadcell_service_deinit();
        }
    } else {
        ESP_LOGW(TAG, "HX711 init failed - continuing without loadcell");
    }

    /* 5. Khoi dong module vong ngoai */
    mission_ctrl_init();
}

void app_start(void)
{
    system_init();

    boot_btn_fsm_t btn;
    boot_btn_init(&btn);

    ESP_LOGI(TAG, "==============================================================");
    ESP_LOGI(TAG, "READY (STANDALONE) | Sensor=%dS | Calib NVS: %s",
             (int)ROBOT_RUN_SENSOR_PROFILE,
             sensor_service_has_valid_calib() ? "VALID" : "DEFAULT (hold BOOT 2s!)");
    ESP_LOGI(TAG, "  * BOOT 1 click : Start line-tracking & payload mission (3s)");
    ESP_LOGI(TAG, "  * BOOT hold 2s : 7-sensor calibration -> save NVS");
    ESP_LOGI(TAG, "==============================================================");

    while (1) {
        btn_event_t evt = boot_btn_poll(&btn);
        if (evt == BTN_EVT_SHORT_PRESS) {
            run_line_mission();
            boot_btn_init(&btn);
            continue;
        } else if (evt == BTN_EVT_LONG_HOLD) {
            run_calibration_workflow();
            boot_btn_init(&btn);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
