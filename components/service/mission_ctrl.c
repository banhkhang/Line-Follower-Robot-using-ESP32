/*
 * ============================================================================
 * mission_ctrl.c - Task Vong ngoai (Core 0) & May trang thai nhiem vu doc lap
 * ============================================================================
 */

#include "mission_ctrl.h"
#include "config.h"
#include "robot_params.h"
#include "drive_service.h"
#include "sensor_service.h"
#include "loadcell_service.h"
#include "outer_estimator.h"
#include "outer_smc.h"
#include "usr_led.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <string.h>

static const char *TAG = "mission_ctrl";

#define OUTER_TASK_STACK_BYTES  5120
#define OUTER_TASK_PRIORITY     14
#define OUTER_TASK_CORE_ID      0

static outer_smc_params_t       s_params;
static bool                     s_inited    = false;
static volatile bool            s_running   = false;
static volatile mission_state_t s_cur_state = MISSION_IDLE;

typedef struct {
    mission_run_cfg_t     cfg;
    TaskHandle_t          caller_task;
    mission_stop_reason_t stop_reason;
} run_job_t;

void mission_ctrl_init(void)
{
    if (s_inited) return;
    outer_smc_get_default_params(&s_params);
    s_inited = true;
}

bool mission_ctrl_is_running(void)
{
    return s_running;
}

mission_state_t mission_ctrl_get_state(void)
{
    return s_cur_state;
}

/* Kiem tra vach cat ngang tram tai hang theo profile 5 hay 7 cam bien */
static bool detect_crossbar(const line_sensor_reading_t *r)
{
    if (r->profile == TCRT_PROFILE_7S) {
        return (r->n_active >= MARKER_MIN_ACTIVE_7S) && (r->total_i > MARKER_TOTAL_I_THR_7S);
    }
    return (r->n_active >= MARKER_MIN_ACTIVE_5S) && (r->total_i > MARKER_TOTAL_I_THR_5S);
}

/* Task thoi gian thuc Vong ngoai (Core 0, dong bo 200 Hz voi Core 1 GPTimer) */
static void outer_ctrl_task(void *pvParameters)
{
    run_job_t *job = (run_job_t *)pvParameters;

    outer_estimator_t est;
    outer_smc_t       smc;
    outer_smc_init(&smc, &s_params);

    /* Doc mau cam bien dau tien de khoi tao bo loc & bo quan sat khong giat */
    line_sensor_reading_t line = {0};
    float e2_init = 0.0f;
    if (sensor_service_read(&line) == ESP_OK && line.line_detected) {
        e2_init = line.e2_smc_m;
    }
    outer_estimator_reset(&est, e2_init, SENSOR_LOOP_TS_S, OBSERVER_LOOP_TS_S);

    /* Dat lai trang thai dong co: tai 0 kg, moc odometry = 0, khoi hanh S-curve */
    drive_service_set_payload_kg(0.0f);
    drive_service_reset_odometry();
    drive_service_start_scurve(smc.p.v_req, ROBOT_SCURVE_DEFAULT_T_S);

    /* Dang ky nhan thong bao dong bo pha 200 Hz (moi 5 ms) tu Core 1 */
    drive_service_register_outer_notify(xTaskGetCurrentTaskHandle(), 1U);

    usr_led_start_run_pattern();

    mission_state_t       mstate      = MISSION_TO_LOAD;
    mission_stop_reason_t stop_reason = STOP_REASON_NONE;
    s_cur_state = mstate;

    int64_t  run_start_us     = esp_timer_get_time();
    uint32_t tick_200hz       = 0;

    float    seg_elapsed_s    = 0.0f;
    float    state_timer_s    = 0.0f;

    /* Bo dem phat hien vach cat ngang & EOL */
    int      marker_hits      = 0;
    bool     marker_seen      = false;
    float    s_marker_entry_m = 0.0f;
    float    s_target_stop_m  = TRACK_LEG1_DIST_M;

    int      eol_hits         = 0;
    float    s_eol_start_m    = 0.0f;
    float    d_goal_adv_m     = ROBOT_SENSOR_OFFSET_M;

    /* Bo dem 2 watchdog dung an toan & nut dung khan cap */
    float    line_loss_timer_s  = 0.0f;
    float    err_over16_timer_s = 0.0f;
    float    boot_hold_s        = 0.0f;

    /* Trang thai can tai trong tai Tram tai hang */
    uint8_t  load_class_kg     = 0;
    float    load_measured_g   = 0.0f;
    float    load_anchor_g     = 0.0f;
    float    load_stable_s     = 0.0f;
    float    load_sum_g        = 0.0f;
    uint32_t load_cnt          = 0;
    bool     load_locked       = false;

    /* Thong ke nhanh in ra UART khi ket thuc */
    float    max_abs_e2_mm     = 0.0f;
    double   sum_sq_e2_mm      = 0.0;
    uint32_t rms_e2_cnt        = 0;

    ESP_LOGI(TAG, "Mission START | profile=%dS v_req=%.2f m/s",
             (int)job->cfg.sensor_profile, smc.p.v_req);

    for (;;) {
        /* Cho dung nhip 5 ms (200 Hz) tu Core 1 GPTimer (timeout 50 ms phong ho) */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        tick_200hz++;

        /* 0. Kiem tra nut dung khan cap (giu BOOT >= 150 ms) */
        if (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
            boot_hold_s += SENSOR_LOOP_TS_S;
            if (boot_hold_s >= 0.15f) {
                drive_service_stop_brake();
                stop_reason = STOP_REASON_MANUAL;
                mstate      = MISSION_STOPPED_FAULT;
                s_cur_state = mstate;
                ESP_LOGW(TAG, "Manual STOP via BOOT -> Short Brake");
                break;
            }
        } else {
            boot_hold_s = 0.0f;
        }

        /* 1. Lay anh chup trang thai vong trong (thread-safe < 1 us) */
        drive_state_t drv;
        drive_service_get_state(&drv);

        /* 2. TANG 200 Hz: Doc cam bien TCRT5000 (ambient-compensated, 2x oversample) */
        if (sensor_service_read(&line) != ESP_OK) {
            drive_service_stop_brake();
            stop_reason = STOP_REASON_FAULT;
            mstate      = MISSION_STOPPED_FAULT;
            s_cur_state = mstate;
            ESP_LOGE(TAG, "Line sensor read failed -> Emergency Brake");
            break;
        }

        bool in_y_zone = (mstate == MISSION_TO_GOAL) &&
                         (drv.s_m >= TRACK_Y_WINDOW_MIN_M) &&
                         (drv.s_m <= TRACK_Y_WINDOW_MAX_M);

        outer_estimator_step_200hz(&est, line.e2_smc_m, line.line_detected,
                                   in_y_zone, SENSOR_LOOP_TS_S);

        /* Phat hien vach cat ngang (Tram tai hang) o 200 Hz - chi mo cong khi gan moc 2.0 m */
        bool is_marker_now = false;
        if (mstate == MISSION_TO_LOAD && drv.s_m >= TRACK_MARKER_WIN_MIN_M) {
            is_marker_now = detect_crossbar(&line);
            if (is_marker_now) {
                if (marker_hits == 0) {
                    s_marker_entry_m = drv.s_m;
                }
                marker_hits++;
                if (marker_hits >= MARKER_CONFIRM_TICKS_200HZ) {
                    marker_seen = true;
                }
            } else if (!marker_seen) {
                marker_hits = 0;
            }
        }

        /* Phat hien dau mut ket thuc cuoi duong (EOL, khong vach ngang) o 200 Hz */
        bool eol_triggered = false;
        if (mstate == MISSION_TO_GOAL && drv.s_m >= TRACK_GOAL_WIN_MIN_M) {
            float eol_thr = (line.profile == TCRT_PROFILE_7S)
                                ? GOAL_EOL_TOT_THRESH_7S
                                : GOAL_EOL_TOT_THRESH_5S;
            if (!line.line_detected || line.total_i < eol_thr) {
                eol_hits++;
                if (eol_hits >= GOAL_EOL_CONFIRM_TICKS) {
                    eol_triggered = true;
                }
            } else {
                eol_hits = 0;
            }
        }

        /* Watchdog an toan: Mat line >= 2.0 s hoac |e2| > 16 mm lien tuc >= 2.0 s */
        bool watchdogs_active = (mstate == MISSION_TO_LOAD ||
                                 mstate == MISSION_LOAD_AXLE_ALIGN ||
                                 mstate == MISSION_TO_GOAL);
        if (watchdogs_active) {
            if (!line.line_detected) {
                line_loss_timer_s += SENSOR_LOOP_TS_S;
            } else {
                line_loss_timer_s = 0.0f;
            }

            if (line.line_detected && fabsf(est.e2_filt) > SAFETY_ERR_LIMIT_M) {
                err_over16_timer_s += SENSOR_LOOP_TS_S;
            } else {
                err_over16_timer_s = 0.0f;
            }

            if (line_loss_timer_s >= SAFETY_LINE_LOST_STOP_S) {
                drive_service_stop_brake();
                stop_reason = STOP_REASON_LINE_LOST_2S;
                mstate      = MISSION_STOPPED_FAULT;
                s_cur_state = mstate;
                ESP_LOGW(TAG, "STOP: Line lost continuously >= %.1f s", SAFETY_LINE_LOST_STOP_S);
                break;
            }
            if (err_over16_timer_s >= SAFETY_ERR_LIMIT_STOP_S) {
                drive_service_stop_brake();
                stop_reason = STOP_REASON_ERR_16MM_2S;
                mstate      = MISSION_STOPPED_FAULT;
                s_cur_state = mstate;
                ESP_LOGW(TAG, "STOP: Tracking error > 16 mm continuously >= %.1f s",
                         SAFETY_ERR_LIMIT_STOP_S);
                break;
            }
        } else {
            line_loss_timer_s  = 0.0f;
            err_over16_timer_s = 0.0f;
        }

        /* Chi chay tang 100 Hz o moi 2 nhip 200 Hz (Ts_in = 10 ms) */
        if ((tick_200hz % OBSERVER_LOOP_DIVIDER) != 0U) {
            continue;
        }
        seg_elapsed_s += OBSERVER_LOOP_TS_S;
        state_timer_s += OBSERVER_LOOP_TS_S;

        /* 3. TANG 100 Hz: Tach nhanh Gauss-Newton (khi o vung nga ba chu Y) */
        if (in_y_zone) {
            outer_estimator_step_y_branch_100hz(&est, line.i_eff, line.total_i,
                                                line.profile,
                                                (load_class_kg == 2U) ? 2U : 1U,
                                                OBSERVER_LOOP_TS_S);
        }

        /* 4. May trang thai nhiem vu & Quy hoach van toc dai v_cmd */
        float v_cmd_out        = 0.0f;
        bool  smc_track_active = false;
        bool  goal_done_exit   = false;

        switch (mstate) {
            case MISSION_TO_LOAD: {
                /* Giam toc muot ve TRACK_ALIGN_SPEED_MPS khi tiep can vach ngang tram tai */
                float d_rem = TRACK_LEG1_DIST_M - drv.s_m;
                v_cmd_out = outer_calc_decel_velocity(d_rem, smc.p.v_req, TRACK_ALIGN_SPEED_MPS,
                                                      TRACK_DECEL_WINDOW_M, TRACK_DECEL_MARGIN_M);
                smc_track_active = true;

                if ((marker_seen && !is_marker_now) || (drv.s_m >= TRACK_LEG1_DIST_M - 0.015f)) {
                    float s_cb_center = marker_seen
                                            ? (0.5f * (s_marker_entry_m + drv.s_m))
                                            : (TRACK_LEG1_DIST_M - ROBOT_SENSOR_OFFSET_M);
                    /* Tien them STATION_STOP_ADVANCE_M (100 mm) ke tu tam vach tai hang de than xe 240 mm
                     * nam gon trong vung tai hang (+-150 mm quanh tam vach) */
                    s_target_stop_m = s_cb_center + STATION_STOP_ADVANCE_M + smc.p.station_trim_m;
                    mstate        = MISSION_LOAD_AXLE_ALIGN;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGI(TAG, "Crossbar passed (center=%.3f m) -> advance +%.0f mm to %.3f m",
                             s_cb_center, STATION_STOP_ADVANCE_M * 1000.0f, s_target_stop_m);
                }
                break;
            }

            case MISSION_LOAD_AXLE_ALIGN: {
                v_cmd_out        = TRACK_ALIGN_SPEED_MPS;
                smc_track_active = true;
                float d_brake    = outer_estimate_brake_lead_m(drv.v_mps, 0.0f);
                if (drv.s_m >= (s_target_stop_m - d_brake)) {
                    drive_service_stop_brake();
                    outer_smc_reset_state(&smc);
                    v_cmd_out        = 0.0f;
                    smc_track_active = false;
                    mstate           = MISSION_WAIT_LOAD;
                    state_timer_s    = 0.0f;
                    load_stable_s    = 0.0f;
                    load_sum_g       = 0.0f;
                    load_cnt         = 0;
                    load_locked      = false;
                    s_cur_state      = mstate;
                    ESP_LOGI(TAG, "Loading station reached (s=%.3f m) -> Waiting load...", drv.s_m);
                }
                break;
            }

            case MISSION_WAIT_LOAD: {
                v_cmd_out        = 0.0f;
                smc_track_active = false;
                drive_service_stop_brake();
                outer_estimator_reset_observer(&est, est.e2_filt);

                /* Doi xe dung han roi doc HX711 khong chan (non-blocking ~10 Hz) */
                if (state_timer_s >= 0.35f && drive_service_is_stopped()) {
                    if (!loadcell_service_is_enabled()) {
                        if (state_timer_s >= 2.0f) {
                            load_measured_g = 1000.0f;
                            load_locked     = true;
                            ESP_LOGW(TAG, "HX711 disabled -> defaulting payload to 1.0 kg");
                        }
                    } else {
                        float g = 0.0f;
                        if (loadcell_service_try_read_single(&g) == ESP_OK) {
                            if (g > LOAD_DETECT_MIN_G) {
                                if (load_cnt == 0 || fabsf(g - load_anchor_g) <= LOAD_STABLE_BAND_G) {
                                    if (load_cnt == 0) {
                                        load_anchor_g = g;
                                        load_stable_s = 0.0f;
                                    }
                                    load_sum_g += g;
                                    load_cnt++;
                                } else {
                                    load_anchor_g = g;
                                    load_sum_g    = g;
                                    load_cnt      = 1;
                                    load_stable_s = 0.0f;
                                }
                            } else {
                                load_cnt      = 0;
                                load_sum_g    = 0.0f;
                                load_stable_s = 0.0f;
                            }
                        }
                        if (load_cnt > 0) {
                            load_stable_s += OBSERVER_LOOP_TS_S;
                            if (load_stable_s >= LOAD_STABLE_HOLD_S) {
                                load_measured_g = load_sum_g / (float)load_cnt;
                                load_locked     = true;
                            }
                        }
                    }

                    if (load_locked) {
                        load_class_kg = (load_measured_g < LOAD_CLASS_SPLIT_G) ? 1U : 2U;
                        float m_load_kg = load_measured_g * 0.001f;

                        /* Cap nhat luat tham so 2-DOF PI vong trong theo khoi luong do thuc */
                        drive_service_set_payload_kg(m_load_kg);
                        drive_service_reset_odometry();
                        outer_smc_reset_state(&smc);
                        outer_estimator_reset_observer(&est, est.e2_filt);
                        drive_service_start_scurve(smc.p.v_req, ROBOT_SCURVE_DEFAULT_T_S);

                        eol_hits      = 0;
                        seg_elapsed_s = 0.0f;
                        state_timer_s = 0.0f;
                        mstate        = MISSION_TO_GOAL;
                        s_cur_state   = mstate;

                        ESP_LOGI(TAG, "Load locked: %.1f g -> Class %u kg (%s branch) | Departing",
                                 load_measured_g, (unsigned)load_class_kg,
                                 (load_class_kg == 1U) ? "LEFT / upper" : "RIGHT / lower");
                    }
                }
                break;
            }

            case MISSION_TO_GOAL: {
                /*
                 * Do xe dung tien them STATION_EXTRA_ADV_M (+45 mm) qua tam vach tai hang,
                 * quang duong danh dinh chang 2 giam tuong ung (TRACK_LEG2_DIST_M - STATION_EXTRA_ADV_M)
                 * de xe van giam toc dung nhip truoc EOL o ca 1 kg & 2 kg.
                 */
                float odom_comp_m   = (load_class_kg == 2U)
                                          ? TRACK_LEG2_2KG_ODOM_COMP_M
                                          : TRACK_LEG2_1KG_ODOM_COMP_M;
                float leg2_odom_ref = (TRACK_LEG2_DIST_M - STATION_EXTRA_ADV_M) + odom_comp_m;
                float d_rem = (leg2_odom_ref - ROBOT_SENSOR_OFFSET_M) - drv.s_m;
                float v_prof = outer_calc_decel_velocity(d_rem, smc.p.v_req, TRACK_ALIGN_SPEED_MPS,
                                                         TRACK_DECEL_WINDOW_M, TRACK_DECEL_MARGIN_M);
                float v_junc = outer_calc_y_junction_speed(drv.s_m, smc.p.v_req);
                v_cmd_out        = fminf(v_prof, v_junc);
                smc_track_active = true;

                if (eol_triggered) {
                    s_eol_start_m = drv.s_m;
                    d_goal_adv_m  = ROBOT_SENSOR_OFFSET_M - GOAL_EOL_PSF_OFFSET_M + smc.p.goal_trim_m;
                    if (d_goal_adv_m < 0.010f) d_goal_adv_m = 0.010f;

                    mstate        = MISSION_GOAL_AXLE_ALIGN;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGI(TAG, "EOL detected at s=%.3f m -> aligning %.1f mm straight",
                             drv.s_m, d_goal_adv_m * 1000.0f);
                } else if (drv.s_m >= TRACK_GOAL_WIN_MAX_M) {
                    drive_service_stop_brake();
                    mstate        = MISSION_STOPPED_GOAL;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGW(TAG, "Goal failsafe odometry reached (s=%.3f m) -> Short Brake", drv.s_m);
                }
                break;
            }

            case MISSION_GOAL_AXLE_ALIGN: {
                /* Khoa lai thang (w_cmd = 0) va tien gon dua TRUC BANH vao dung vach dich */
                v_cmd_out        = TRACK_ALIGN_SPEED_MPS;
                smc_track_active = false;
                float crept_m    = drv.s_m - s_eol_start_m;
                float d_brake    = outer_estimate_brake_lead_m(drv.v_mps, load_measured_g * 0.001f);
                float adv_stop_m = fmaxf(0.005f, d_goal_adv_m - d_brake);

                if (crept_m >= adv_stop_m || state_timer_s >= 1.2f) {
                    drive_service_stop_brake();
                    mstate        = MISSION_STOPPED_GOAL;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGI(TAG, "GOAL REACHED! s=%.3f m (creep=%.1f mm) -> holding 2.0 s",
                             drv.s_m, crept_m * 1000.0f);
                } else {
                    drive_service_set_chassis_vel(TRACK_ALIGN_SPEED_MPS, 0.0f);
                }
                break;
            }

            case MISSION_STOPPED_GOAL: {
                v_cmd_out        = 0.0f;
                smc_track_active = false;
                drive_service_stop_brake();
                if (state_timer_s >= GOAL_STOP_HOLD_S) {
                    stop_reason    = STOP_REASON_GOAL_DONE;
                    goal_done_exit = true;
                }
                break;
            }

            default:
                drive_service_stop_brake();
                break;
        }

        /* 5. Cap nhat Bo quan sat Luenberger (100 Hz) & Bo lap lich SMC thich nghi */
        float v_eff_ctrl = drv.scurve_active ? fminf(v_cmd_out, drv.v_cmd_eff_mps) : v_cmd_out;

        outer_estimator_step_observer_100hz(&est, drv.v_mps, drv.w_rads,
                                            v_eff_ctrl, smc_track_active,
                                            line.line_detected, OBSERVER_LOOP_TS_S);

        outer_smc_step_adaptive(&smc,
                                est.e2_filt, est.e3_hat, est.c_hat,
                                v_eff_ctrl, seg_elapsed_s,
                                drv.v_a_ref_mps, drv.v_a_mps,
                                drv.v_b_ref_mps, drv.v_b_mps,
                                smc_track_active, line.line_detected,
                                OBSERVER_LOOP_TS_S);

        /* Gui lenh (v_cmd, w_cmd) xuong vong trong 200 Hz (Core 1) */
        if (smc_track_active) {
            drive_service_set_chassis_vel(v_cmd_out, smc.w_cmd);
        }

        /* 6. Thong ke sai so bam line de bao cao ngan qua UART */
        if (smc_track_active && seg_elapsed_s > 0.60f && line.line_detected) {
            float e2_mm = fabsf(est.e2_filt) * 1000.0f;
            if (e2_mm > max_abs_e2_mm) max_abs_e2_mm = e2_mm;
            sum_sq_e2_mm += (double)(e2_mm * e2_mm);
            rms_e2_cnt++;
        }

        if (goal_done_exit) {
            break;
        }
    }

    /* Dung an toan: huy dang ky notify, phanh den khi dung han roi nha Coast */
    drive_service_register_outer_notify(NULL, 0U);
    drive_service_stop_brake();
    for (int i = 0; i < 40 && !drive_service_is_stopped(); i++) {
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    drive_service_stop_coast();
    usr_led_set(false);

    uint32_t dur_ms = (uint32_t)((esp_timer_get_time() - run_start_us) / 1000LL);
    float rms_mm    = (rms_e2_cnt > 0) ? (float)sqrt(sum_sq_e2_mm / (double)rms_e2_cnt) : 0.0f;
    ESP_LOGI(TAG, "Mission END | reason=%d dur=%lu ms | load=%.0fg (%ukg) | max|e2|=%.2f mm RMS=%.2f mm",
             (int)stop_reason, (unsigned long)dur_ms, load_measured_g,
             (unsigned)load_class_kg, max_abs_e2_mm, rms_mm);

    job->stop_reason = stop_reason;
    s_cur_state      = MISSION_IDLE;
    s_running        = false;
    xTaskNotifyGive(job->caller_task);
    vTaskDelete(NULL);
}

mission_stop_reason_t mission_ctrl_run_blocking(const mission_run_cfg_t *cfg)
{
    if (cfg == NULL) return STOP_REASON_FAULT;
    if (!s_inited) mission_ctrl_init();

    s_running   = true;
    s_cur_state = MISSION_IDLE;

    /* Dem nguoc truoc khi chay (nhan BOOT de huy ngay) */
    if (cfg->countdown_rem_ms > 0) {
        ESP_LOGI(TAG, "Countdown %lu ms before launch (press BOOT to abort)...",
                 (unsigned long)cfg->countdown_rem_ms);
        usr_led_start_blink(150);
        uint32_t waited = 0;
        while (waited < cfg->countdown_rem_ms) {
            if (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
                usr_led_set(false);
                s_running = false;
                ESP_LOGW(TAG, "Launch aborted during countdown");
                while (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                return STOP_REASON_MANUAL;
            }
            vTaskDelay(pdMS_TO_TICKS(25));
            waited += 25;
        }
        usr_led_set(false);
    }

    run_job_t job;
    memset(&job, 0, sizeof(job));
    job.cfg         = *cfg;
    job.caller_task = xTaskGetCurrentTaskHandle();
    job.stop_reason = STOP_REASON_FAULT;

    BaseType_t ok = xTaskCreatePinnedToCore(
        outer_ctrl_task,
        "outer_smc_200hz",
        OUTER_TASK_STACK_BYTES,
        &job,
        OUTER_TASK_PRIORITY,
        NULL,
        OUTER_TASK_CORE_ID
    );
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create outer_ctrl_task!");
        s_running = false;
        return STOP_REASON_FAULT;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return job.stop_reason;
}
