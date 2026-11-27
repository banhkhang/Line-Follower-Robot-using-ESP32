/*
 * ============================================================================
 * app.c - Lop ung dung chinh cua Line Follower Robot (He A + He B + Standalone)
 * ============================================================================
 *
 * KIEN TRUC TICH HOP 2 HE THONG TRONG 1 BAN FIRMWARE DUY NHAT (Runtime Profile):
 *
 *  [Khoi dong / Nhan nut EN (Reset)]:
 *    1) Khoa cau H o trang thai Coast (Hi-Z) an toan ngay lap tuc.
 *    2) Khoi tao LED1, NVS, Wi-Fi SoftAP ("LineRobot-AP", IP 192.168.4.1).
 *    3) Mo cua so OTA (15 s; nhan BOOT de bo qua ngay). Giu SoftAP bat sau OTA.
 *    4) Khoi chay vong trong `drive_service` (Core 1, 200 Hz, 2-DOF PI).
 *    5) Nap hieu chuan 7 mat TCRT5000 tu NVS (bat profile 5S an toan voi Wi-Fi).
 *    6) Khoi tao & Tare Loadcell HX711 (nhay LED1 2 s).
 *    7) Vao che do mac dinh IDLE (Mode 1 - Wi-Fi ON cho lenh MATLAB, dong thoi
 *       lang nghe nut BOOT cho Mode 2 & Hieu chuan 7 mat):
 *
 *  [Che do 1 - Ket noi MATLAB qua Wi-Fi]:
 *    - Lenh START_A (He A): Dung 5 cam bien (S2..S6), giu Wi-Fi ON, dem nguoc 3 s,
 *      chay vong ngoai SMC tren ESP32 va stream truc tiep du lieu 100 Hz ve MATLAB.
 *    - Lenh START_B (He B): Gui ACK -> tat Wi-Fi -> chuyen profile 7 cam bien
 *      (S1..S7) -> cap phat SRAM -> dem nguoc max(3s, t_switch) -> chay & log
 *      vao SRAM. Khi dung o BAT KY 1 trong 3 kich ban [Stop] (mat line 2s /
 *      sai so > 16mm 2s / dung tai dich du 2s): tat ADC2, bat lai Wi-Fi AP,
 *      xac nhan ket noi MATLAB (PING) va cho lenh BURST_REQ de day toan bo SRAM!
 *
 *  [Che do 2 - Chay doc lap / Hieu chuan bang nut BOOT (khi dang IDLE)]:
 *    - Nhan BOOT 1 lan (Single click) : Tat Wi-Fi -> 7 cam bien -> dem nguoc
 *      max(3s, t_switch) -> chay do line doc lap (KHONG log data).
 *    - Nhan BOOT 2 lan (Double click) : Tat Wi-Fi -> 5 cam bien -> dem nguoc
 *      3s -> chay do line doc lap (KHONG log data).
 *    - Giu BOOT >= 2.0 s (Long hold)  : Hieu chuan 7 cam bien & luu NVS:
 *        * LED1 nhap nhay canh bao: BAT 1s, TAT 1s x 3 lan truoc khi tat Wi-Fi.
 *        * Tat Wi-Fi, chuyen profile 7 mat, LED1 SANG LIEN TUC trong luc quet.
 *        * Nhan BOOT them 1 lan khi quet xong -> luu NVS, tat LED1, bat lai Wi-Fi.
 * ============================================================================
 */

#include "app.h"
#include "ota.h"
#include "config.h"
#include "robot_params.h"
#include "wifi_ap.h"
#include "usr_led.h"
#include "drive_service.h"
#include "sensor_service.h"
#include "loadcell_service.h"
#include "data_logger.h"
#include "mission_ctrl.h"
#include "matlab_link.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

typedef enum {
    BTN_EVT_NONE = 0,
    BTN_EVT_SINGLE_CLICK,
    BTN_EVT_DOUBLE_CLICK,
    BTN_EVT_LONG_HOLD
} btn_event_t;

/* -------------------------------------------------------------------------- */
/* Trinh phat hien cu chi nut BOOT (khong chan, goi moi 10 ms trong IDLE)     */
/* -------------------------------------------------------------------------- */
typedef struct {
    bool     pressed;
    bool     long_fired;
    int64_t  press_start_ms;
    int64_t  last_release_ms;
    uint8_t  click_count;
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
    fsm->pressed         = false;
    fsm->long_fired      = false;
    fsm->press_start_ms  = 0;
    fsm->last_release_ms = 0;
    fsm->click_count     = 0;
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
            fsm->long_fired  = true;
            fsm->click_count = 0;
            return BTN_EVT_LONG_HOLD;
        }
    } else {
        if (fsm->pressed) {
            int64_t dur = t - fsm->press_start_ms;
            fsm->pressed = false;
            if (!fsm->long_fired && dur >= (int64_t)BTN_DEBOUNCE_MS) {
                fsm->click_count++;
                fsm->last_release_ms = t;
                if (fsm->click_count >= 2) {
                    fsm->click_count = 0;
                    return BTN_EVT_DOUBLE_CLICK;
                }
            }
        } else if (fsm->click_count == 1 && (t - fsm->last_release_ms) >= (int64_t)BTN_DOUBLE_CLICK_MS) {
            fsm->click_count = 0;
            return BTN_EVT_SINGLE_CLICK;
        }
    }
    return BTN_EVT_NONE;
}

/* -------------------------------------------------------------------------- */
/* Chuyen doi Wi-Fi <-> Profile cam bien an toan (tranh xung dot ADC2)        */
/* -------------------------------------------------------------------------- */

/* Tat MATLAB link & Wi-Fi, chuyen TCRT5000 sang profile mong muon; tra ve t_switch [ms] */
static uint32_t switch_to_offline_profile(tcrt_profile_t target_profile)
{
    int64_t t0 = esp_timer_get_time();

    matlab_link_stop();
    wifi_ap_off();
    vTaskDelay(pdMS_TO_TICKS(50)); /* Cho phan cung RF nha hoan toan ADC2 */

    esp_err_t err = sensor_service_set_profile(target_profile);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch sensor profile to %dS: %s",
                 (int)target_profile, esp_err_to_name(err));
    }

    uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000LL);
    ESP_LOGI(TAG, "Switched offline to %d-sensor profile in %lu ms",
             (int)target_profile, (unsigned long)elapsed_ms);
    return elapsed_ms;
}

/* Nha ADC2 (ve 5S), bat lai Wi-Fi SoftAP & khoi dong lai MATLAB UDP link */
static void restore_online_wifi_mode(app_link_state_t target_state)
{
    /* Bat buoc nha ADC2 truoc khi bat Wi-Fi! */
    sensor_service_set_profile(TCRT_PROFILE_5S);

    esp_err_t err = wifi_ap_on(ROBOT_WIFI_SSID, ROBOT_WIFI_PASS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart Wi-Fi SoftAP: %s", esp_err_to_name(err));
    }

    matlab_link_set_app_state(target_state);
    if (matlab_link_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart MATLAB UDP link");
    }
}

/* -------------------------------------------------------------------------- */
/* Quy trinh hieu chuan 7 cam bien & luu NVS (co chuoi LED1 1s ON/OFF x 3)    */
/* -------------------------------------------------------------------------- */
static void run_calibration_workflow(void)
{
    ESP_LOGI(TAG, "=== ENTERING 7-SENSOR CALIBRATION WORKFLOW ===");
    matlab_link_set_app_state(APP_STATE_CALIB);

    /* Doi nguoi dung tha nut BOOT sau khi giu 2s */
    while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* Truoc khi tat Wi-Fi: LED1 BAT 1s, TAT 1s lap lai 3 lan theo yeu cau */
    for (int i = 0; i < CALIB_PRELUDE_CYCLES; i++) {
        ESP_LOGI(TAG, "Calib prelude %d/%d (LED1 ON 1s / OFF 1s)", i + 1, CALIB_PRELUDE_CYCLES);
        usr_led_set(true);
        vTaskDelay(pdMS_TO_TICKS(CALIB_PRELUDE_ON_MS));
        usr_led_set(false);
        vTaskDelay(pdMS_TO_TICKS(CALIB_PRELUDE_OFF_MS));
    }

    /* Tat Wi-Fi -> chuyen sang 7 cam bien -> quet (LED1 sang lien tuc -> xong tat LED1) */
    switch_to_offline_profile(TCRT_PROFILE_7S);
    esp_err_t err = sensor_service_calibrate_sweep();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "7-sensor calibration saved to NVS successfully");
    } else {
        ESP_LOGE(TAG, "Calibration failed (%s)", esp_err_to_name(err));
    }

    /* Bat lai Wi-Fi & tro ve trang thai IDLE */
    restore_online_wifi_mode(APP_STATE_IDLE);
}

/* -------------------------------------------------------------------------- */
/* Chay He A (5S + Live Stream Wi-Fi lien tuc voi MATLAB)                     */
/* -------------------------------------------------------------------------- */
static void run_system_a(void)
{
    ESP_LOGI(TAG, "=== SYSTEM A: 5 SENSORS + LIVE WI-FI STREAM ===");
    sensor_service_set_profile(TCRT_PROFILE_5S);
    matlab_link_set_app_state(APP_STATE_RUNNING_A);

    mission_run_cfg_t cfg = {
        .mode             = RUN_MODE_SYS_A_5S_LIVE,
        .sensor_profile   = TCRT_PROFILE_5S,
        .enable_sram_log  = false,
        .live_cb          = matlab_link_send_live_sample,
        .countdown_rem_ms = RUN_COUNTDOWN_MIN_MS  /* Dem nguoc dung 3.0 s */
    };

    robot_run_summary_t summary;
    mission_ctrl_run_blocking(&cfg, &summary);

    matlab_link_set_app_state(APP_STATE_IDLE);
    matlab_link_send_status_now();
}

/* -------------------------------------------------------------------------- */
/* Chay He B (7S + Log SRAM + Bat Wi-Fi Burst khi dung o ca 3 kich ban)       */
/* -------------------------------------------------------------------------- */
static void run_system_b(void)
{
    ESP_LOGI(TAG, "=== SYSTEM B: 7 SENSORS + SRAM LOG + WI-FI BURST ===");
    /* Cho 120 ms de goi PKT_ACK gui xong qua Wi-Fi truoc khi ngat radio */
    vTaskDelay(pdMS_TO_TICKS(120));

    uint32_t t_switch_ms = switch_to_offline_profile(TCRT_PROFILE_7S);

    /* Cap phat bo dem SRAM (luon giu >= 88 KB DRAM trong cho pha bat Wi-Fi burst) */
    bool sram_ok = (data_logger_prepare(SYSTEM_B_LOG_MAX_SAMPLES) == ESP_OK);

    /* Doi max(3s, thoi gian chuyen sang profile 7 cam bien) */
    uint32_t rem_ms = (t_switch_ms < RUN_COUNTDOWN_MIN_MS)
                          ? (RUN_COUNTDOWN_MIN_MS - t_switch_ms)
                          : 0U;

    mission_run_cfg_t cfg = {
        .mode             = RUN_MODE_SYS_B_7S_BURST,
        .sensor_profile   = TCRT_PROFILE_7S,
        .enable_sram_log  = sram_ok,
        .live_cb          = NULL,
        .countdown_rem_ms = rem_ms
    };

    robot_run_summary_t summary;
    robot_stop_reason_t reason = mission_ctrl_run_blocking(&cfg, &summary);

    /*
     * Trong CA 3 kich ban dung (mat line 2s / |e2|>16mm 2s / dung dich du 2s)
     * va ca khi dung thu cong: bat lai Wi-Fi, xac nhan ket noi MATLAB va cho
     * lenh BURST_REQ tu MATLAB de truyen toan bo du lieu SRAM!
     */
    ESP_LOGI(TAG, "System B stopped (reason=%d, samples=%lu). Re-enabling Wi-Fi for MATLAB burst...",
             (int)reason, (unsigned long)data_logger_count());

    restore_online_wifi_mode(APP_STATE_WAIT_BURST);
    usr_led_start_blink(500); /* Nhay cham 0.5s bao hieu dang cho MATLAB nhan burst */

    while (1) {
        matlab_action_t act = matlab_link_take_action();
        if (act == MATLAB_ACT_BURST_DONE) {
            ESP_LOGI(TAG, "Burst transfer completed by MATLAB -> freeing SRAM & returning IDLE");
            break;
        }
        /* Neu nguoi dung giu BOOT >= 1s trong luc cho burst -> huy cho va ve IDLE */
        if (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            if (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
                ESP_LOGW(TAG, "BOOT held -> aborting burst wait & returning IDLE");
                while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    usr_led_set(false);
    data_logger_free();
    matlab_link_set_app_state(APP_STATE_IDLE);
    matlab_link_send_status_now();
}

/* -------------------------------------------------------------------------- */
/* Chay Doc lap bang nut BOOT (Mode 2: 7S hoac 5S, KHONG log data)            */
/* -------------------------------------------------------------------------- */
static void run_standalone(tcrt_profile_t profile)
{
    ESP_LOGI(TAG, "=== STANDALONE MODE (NO LOG): %d SENSORS ===", (int)profile);
    uint32_t t_switch_ms = switch_to_offline_profile(profile);

    uint32_t rem_ms = (t_switch_ms < RUN_COUNTDOWN_MIN_MS)
                          ? (RUN_COUNTDOWN_MIN_MS - t_switch_ms)
                          : 0U;

    mission_run_cfg_t cfg = {
        .mode             = (profile == TCRT_PROFILE_7S)
                                ? RUN_MODE_STANDALONE_7S
                                : RUN_MODE_STANDALONE_5S,
        .sensor_profile   = profile,
        .enable_sram_log  = false,
        .live_cb          = NULL,
        .countdown_rem_ms = rem_ms
    };

    robot_run_summary_t summary;
    mission_ctrl_run_blocking(&cfg, &summary);

    /* Sau khi dung xe: bat lai Wi-Fi AP ve che do IDLE san sang cho luot tiep theo */
    restore_online_wifi_mode(APP_STATE_IDLE);
}

/* -------------------------------------------------------------------------- */
/* Khoi tao he thong & Vong lap dieu phoi chinh                               */
/* -------------------------------------------------------------------------- */
static void system_init(void)
{
    /* 0. AN TOAN PHAN CUNG: Khoa cau H o trang thai Coast (Hi-Z) NGAY KHI BOOT */
    drive_service_lock_motors();

    /* Xac nhan ban firmware hop le neu vua cap nhat qua OTA */
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
    }

    ESP_LOGW("BOOT", "========================================");
    ESP_LOGW("BOOT", "LineRobot ESP32 Boot (reset reason=%d)", (int)esp_reset_reason());
    ESP_LOGW("BOOT", "========================================");

    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_lwip", ESP_LOG_WARN);

    usr_led_init();
    ESP_ERROR_CHECK(wifi_ap_init_base());

    /* 1. Cua so OTA (15s, nhan BOOT de bo qua ngay; giu SoftAP bat sau OTA) */
    ota_boot_window();

    /* 2. Khoi dong vong trong 200 Hz doc quyen tren Core 1 */
    ESP_ERROR_CHECK(drive_service_start());

    /* 3. Khoi dong cam bien do line (profile 5S tuong thich Wi-Fi, tu nap calib 7S tu NVS) */
    if (sensor_service_init(TCRT_PROFILE_5S) != ESP_OK) {
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

    /* 5. Khoi dong module vong ngoai & ken UDP MATLAB */
    mission_ctrl_init();
    matlab_link_set_app_state(APP_STATE_IDLE);
    ESP_ERROR_CHECK(matlab_link_start());
}

void app_start(void)
{
    system_init();

    boot_btn_fsm_t btn;
    boot_btn_init(&btn);

    ESP_LOGI(TAG, "==============================================================");
    ESP_LOGI(TAG, "READY (IDLE) | Wi-Fi AP: '%s' (192.168.4.1:%d) | Calib NVS: %s",
             ROBOT_WIFI_SSID, MATLAB_CMD_PORT,
             sensor_service_has_valid_calib() ? "VALID" : "DEFAULT (hold BOOT 2s!)");
    ESP_LOGI(TAG, "  * MATLAB Wi-Fi : Send START_A (5S Live) or START_B (7S Burst)");
    ESP_LOGI(TAG, "  * BOOT 1 click : Standalone 7-sensor run (no log)");
    ESP_LOGI(TAG, "  * BOOT 2 clicks: Standalone 5-sensor run (no log)");
    ESP_LOGI(TAG, "  * BOOT hold 2s : 7-sensor calibration -> save NVS");
    ESP_LOGI(TAG, "==============================================================");

    while (1) {
        /* 1. Kiem tra lenh tu MATLAB qua Wi-Fi */
        matlab_action_t act = matlab_link_take_action();
        if (act == MATLAB_ACT_START_A) {
            run_system_a();
            boot_btn_init(&btn);
            continue;
        } else if (act == MATLAB_ACT_START_B) {
            run_system_b();
            boot_btn_init(&btn);
            continue;
        }

        /* 2. Kiem tra nut BOOT (Che do 2 doc lap & Hieu chuan 7 mat) */
        btn_event_t evt = boot_btn_poll(&btn);
        if (evt == BTN_EVT_SINGLE_CLICK) {
            run_standalone(TCRT_PROFILE_7S);
            boot_btn_init(&btn);
            continue;
        } else if (evt == BTN_EVT_DOUBLE_CLICK) {
            run_standalone(TCRT_PROFILE_5S);
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