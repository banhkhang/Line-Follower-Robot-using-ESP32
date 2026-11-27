/*
 * ============================================================================
 * mission_ctrl.c - Trien khai Task Vong ngoai (Core 0) & May trang thai nhiem vu
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

#define OUTER_TASK_STACK_BYTES  6144
#define OUTER_TASK_PRIORITY     14
#define OUTER_TASK_CORE_ID      0

static portMUX_TYPE          s_param_lock   = portMUX_INITIALIZER_UNLOCKED;
static outer_smc_params_t    s_params;
static bool                  s_inited       = false;
static volatile bool         s_running      = false;
static volatile bool         s_stop_req     = false;
static volatile mission_state_t s_cur_state = MISSION_IDLE;

/* Ngu canh chuyen vao task vong ngoai */
typedef struct {
    mission_run_cfg_t   cfg;
    TaskHandle_t        caller_task;
    robot_stop_reason_t stop_reason;
    robot_run_summary_t summary;
} run_job_t;

static inline int16_t clamp_i16(float x)
{
    if (x > 32767.0f)  return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lroundf(x);
}

static inline uint16_t clamp_u16(float x)
{
    if (x <= 0.0f)     return 0;
    if (x >= 65535.0f) return 65535;
    return (uint16_t)lroundf(x);
}

void mission_ctrl_init(void)
{
    if (s_inited) return;
    portENTER_CRITICAL(&s_param_lock);
    outer_smc_get_default_params(&s_params);
    portEXIT_CRITICAL(&s_param_lock);
    s_inited = true;
}

void mission_ctrl_set_params(const outer_smc_params_t *params)
{
    if (params == NULL) return;
    if (!s_inited) mission_ctrl_init();

    /* Dung outer_smc_set_params tam de ap dung bo kep gioi han an toan */
    outer_smc_t tmp;
    outer_smc_set_params(&tmp, params);

    portENTER_CRITICAL(&s_param_lock);
    s_params = tmp.p;
    portEXIT_CRITICAL(&s_param_lock);

    ESP_LOGI(TAG, "Params updated: v=%.2f beta=%.4f k=%.3f phi=%.4f ki=%.2f Ts=[%.0f..%.0f]ms",
             tmp.p.v_req, tmp.p.beta_smc, tmp.p.k_smc, tmp.p.phi_bl, tmp.p.ki_smc,
             tmp.p.ts_min_s * 1000.0f, tmp.p.ts_max_s * 1000.0f);
}

void mission_ctrl_get_params(outer_smc_params_t *out)
{
    if (out == NULL) return;
    if (!s_inited) mission_ctrl_init();
    portENTER_CRITICAL(&s_param_lock);
    *out = s_params;
    portEXIT_CRITICAL(&s_param_lock);
}

void mission_ctrl_request_stop(void)
{
    s_stop_req = true;
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

/* Dong goi 1 mau 52 byte (dung cho ca Live Stream He A va SRAM Log He B) */
static void pack_log_sample(robot_log_sample_t *s,
                            uint32_t t_ms,
                            const outer_estimator_t *est,
                            const outer_smc_t *smc,
                            const drive_state_t *drv,
                            const line_sensor_reading_t *line,
                            mission_state_t mstate,
                            bool smc_updated,
                            bool marker_seen,
                            uint8_t load_class)
{
    s->t_ms            = t_ms;
    s->e2_raw_cmm      = clamp_i16(est->last_e2_raw * 100000.0f);  /* m -> 0.01 mm */
    s->e2_filt_cmm     = clamp_i16(est->e2_filt     * 100000.0f);
    s->e3_hat_cdeg     = clamp_i16(est->e3_hat * (18000.0f / 3.14159265f)); /* rad -> 0.01 deg */
    s->c_hat_mrad      = clamp_i16(est->c_hat       * 1000.0f);
    s->sigma_cmm       = clamp_i16(smc->sigma       * 100000.0f);
    s->z_int_cmm       = clamp_i16(smc->z_int       * 100000.0f);
    s->v_cmd_mms       = clamp_i16(drv->v_cmd_eff_mps * 1000.0f);
    s->w_cmd_mrads     = clamp_i16(smc->w_cmd       * 1000.0f);
    s->v_act_mms       = clamp_i16(drv->v_mps       * 1000.0f);
    s->w_act_mrads     = clamp_i16(drv->w_rads      * 1000.0f);
    s->va_ref_mms      = clamp_i16(drv->v_a_ref_mps * 1000.0f);
    s->va_act_mms      = clamp_i16(drv->v_a_mps     * 1000.0f);
    s->vb_ref_mms      = clamp_i16(drv->v_b_ref_mps * 1000.0f);
    s->vb_act_mms      = clamp_i16(drv->v_b_mps     * 1000.0f);
    s->pwm_a_cpct      = clamp_i16(drv->pwm_a_pct   * 100.0f);
    s->pwm_b_cpct      = clamp_i16(drv->pwm_b_pct   * 100.0f);
    s->s_enc_mm        = clamp_u16(drv->s_m         * 1000.0f);
    s->total_i_m       = clamp_u16(line->total_i    * 1000.0f);
    s->inner_jitter_us = drv->dt_us_err;

    for (int i = 0; i < LINE_SENSOR_MAX_COUNT; i++) {
        float n = line->norm[i];
        if (n < 0.0f)   n = 0.0f;
        if (n > 100.0f) n = 100.0f;
        s->norm_s[i] = (uint8_t)lroundf(n);
    }

    uint8_t flags = ((uint8_t)mstate) & LOG_FLAG_STATE_MASK;
    if (line->line_detected) flags |= LOG_FLAG_LINE_OK;
    if (est->y_gn_active)    flags |= LOG_FLAG_Y_GN_ACTIVE;
    if (smc_updated)         flags |= LOG_FLAG_SMC_UPDATED;
    if (marker_seen)         flags |= LOG_FLAG_MARKER_SEEN;
    s->state_flags = flags;

    float dt_ms = smc->last_dt_smc_s * 1000.0f;
    if (dt_ms < 0.0f)   dt_ms = 0.0f;
    if (dt_ms > 255.0f) dt_ms = 255.0f;
    s->dt_smc_ms  = (uint8_t)lroundf(dt_ms);
    s->load_class = load_class;
}

/* Task thoi gian thuc Vong ngoai (Core 0, dong bo 200 Hz voi Core 1 GPTimer) */
static void outer_ctrl_task(void *pvParameters)
{
    run_job_t *job = (run_job_t *)pvParameters;

    outer_smc_params_t p_local;
    mission_ctrl_get_params(&p_local);

    outer_estimator_t est;
    outer_smc_t       smc;
    outer_smc_init(&smc, &p_local);

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

    mission_state_t     mstate          = MISSION_TO_LOAD;
    robot_stop_reason_t stop_reason     = STOP_REASON_NONE;
    s_cur_state = mstate;

    int64_t  run_start_us     = esp_timer_get_time();
    uint32_t tick_200hz       = 0;
    uint32_t tick_100hz       = 0;
    uint32_t wait_log_div     = 0;

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

    /* Bo dem 2 watchdog dung an toan [Stop] */
    float    line_loss_timer_s = 0.0f;
    float    err_over16_timer_s= 0.0f;
    float    boot_hold_s       = 0.0f;

    /* Trang thai can tai trong tai Tram tai hang */
    uint8_t  load_class_kg     = 0;
    float    load_measured_g   = 0.0f;
    float    load_anchor_g     = 0.0f;
    float    load_stable_s     = 0.0f;
    float    load_sum_g        = 0.0f;
    uint32_t load_cnt          = 0;
    bool     load_locked       = false;

    /* Thong ke KPI tom tat */
    float    max_abs_e2_mm     = 0.0f;
    double   sum_sq_e2_mm      = 0.0;
    uint32_t rms_e2_cnt        = 0;
    float    station_stop_s_m  = 0.0f;
    float    goal_stop_s_m     = 0.0f;
    float    goal_creep_mm     = 0.0f;
    double   sum_dt_smc_ms     = 0.0;
    uint32_t smc_update_cnt    = 0;
    uint32_t y_gn_calls        = 0;

    ESP_LOGI(TAG, "Mission START | mode=%d profile=%dS v_req=%.2f m/s",
             (int)job->cfg.mode, (int)job->cfg.sensor_profile, smc.p.v_req);

    for (;;) {
        /* Cho dung nhip 5 ms (200 Hz) tu Core 1 GPTimer (timeout 50 ms phong ho) */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        tick_200hz++;

        /* 0. Kiem tra lenh dung khan cap (MATLAB STOP hoac nhan giu BOOT >= 150 ms) */
        if (gpio_get_level(PIN_BOOT_BUTTON) == 0) {
            boot_hold_s += SENSOR_LOOP_TS_S;
            if (boot_hold_s >= 0.15f) {
                s_stop_req = true;
            }
        } else {
            boot_hold_s = 0.0f;
        }

        if (s_stop_req) {
            drive_service_stop_brake();
            stop_reason = STOP_REASON_MANUAL;
            mstate      = MISSION_STOPPED_FAULT;
            s_cur_state = mstate;
            ESP_LOGW(TAG, "Manual STOP requested -> Short Brake");
            break;
        }

        /* Cap nhat nhanh tham so neu MATLAB vua gui CMD_SET_PARAMS trong luc chay He A */
        if (job->cfg.mode == RUN_MODE_SYS_A_5S_LIVE) {
            outer_smc_params_t p_new;
            mission_ctrl_get_params(&p_new);
            smc.p = p_new;
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

        /* Watchdog [Stop]: Mat line >= 2.0 s hoac |e2| > 16 mm lien tuc >= 2.0 s */
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
        tick_100hz++;
        seg_elapsed_s += OBSERVER_LOOP_TS_S;
        state_timer_s += OBSERVER_LOOP_TS_S;

        /* 3. TANG 100 Hz: Tach nhanh Gauss-Newton (khi o vung nga ba chu Y) */
        if (in_y_zone) {
            outer_estimator_step_y_branch_100hz(&est, line.i_eff, line.total_i,
                                                line.profile,
                                                (load_class_kg == 2U) ? 2U : 1U,
                                                OBSERVER_LOOP_TS_S);
            if (est.y_gn_active) {
                y_gn_calls++;
            }
        }

        /* 4. May trang thai nhiem vu & Quy hoach van toc dai v_cmd */
        float v_cmd_out       = 0.0f;
        bool  smc_track_active = false;
        bool  goal_done_exit   = false;

        switch (mstate) {
            case MISSION_TO_LOAD: {
                /* Giam toc muot khi tiep can vach ngang tram tai (s = 2.000 m) */
                float d_rem = TRACK_LEG1_DIST_M - drv.s_m;
                v_cmd_out = outer_calc_decel_velocity(d_rem, smc.p.v_req, ROBOT_V_MIN_MPS,
                                                      TRACK_DECEL_WINDOW_M, TRACK_DECEL_MARGIN_M);
                smc_track_active = true;

                /*
                 * Chuyen sang pha can chinh truc banh ngay khi cam bien vua qua mep sau
                 * vach cat ngang (suon ra cua marker) hoac cham fail-safe odometry.
                 */
                if ((marker_seen && !is_marker_now) || (drv.s_m >= TRACK_LEG1_DIST_M - 0.015f)) {
                    float s_cb_center = marker_seen
                                            ? (0.5f * (s_marker_entry_m + drv.s_m))
                                            : (TRACK_LEG1_DIST_M - ROBOT_SENSOR_OFFSET_M);
                    float d_brake = outer_estimate_brake_lead_m(drv.v_mps, 0.0f);
                    /* Trung diem truc banh cach day cam bien d1 = 55 mm ve phia sau */
                    s_target_stop_m = s_cb_center + ROBOT_SENSOR_OFFSET_M
                                    + smc.p.station_trim_m - d_brake;
                    mstate        = MISSION_LOAD_AXLE_ALIGN;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGI(TAG, "Crossbar passed (center=%.3f m) -> align drive axle to %.3f m",
                             s_cb_center, s_target_stop_m);
                }
                break;
            }

            case MISSION_LOAD_AXLE_ALIGN: {
                v_cmd_out        = ROBOT_V_MIN_MPS;
                smc_track_active = true;
                if (drv.s_m >= s_target_stop_m) {
                    drive_service_stop_brake();
                    outer_smc_reset_state(&smc);
                    v_cmd_out        = 0.0f;
                    smc_track_active = false;
                    station_stop_s_m = drv.s_m;
                    mstate           = MISSION_WAIT_LOAD;
                    state_timer_s    = 0.0f;
                    load_stable_s    = 0.0f;
                    load_sum_g       = 0.0f;
                    load_cnt         = 0;
                    load_locked      = false;
                    s_cur_state      = mstate;
                    ESP_LOGI(TAG, "Loading station reached (axle s=%.3f m, err=%+.1f mm) -> Waiting load...",
                             drv.s_m, (drv.s_m - TRACK_LEG1_DIST_M) * 1000.0f);
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
                        /* Du phong neu test khong gan HX711: cho 2.0s roi mac dinh 1 kg */
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
                                    /* Khoi hang dang dao dong / tay nguoi dang dat -> reset cua so */
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

                        ESP_LOGI(TAG, "Load locked: %.1f g -> Class %u kg (%s branch) | Updating 2-DOF PI & departing",
                                 load_measured_g, (unsigned)load_class_kg,
                                 (load_class_kg == 1U) ? "LEFT / upper" : "RIGHT / lower");
                    }
                }
                break;
            }

            case MISSION_TO_GOAL: {
                /*
                 * Cam bien cham dau mut cuoi duong (X = 3000 mm) tai s_axle ~ 4.654 m (1 kg)
                 * va ~4.689 m (2 kg do nen lop nhe ~0.7%). Bu theo tai de thoi gian bo creep-speed
                 * truoc EOL ngan gon nhu nhau o ca 1 kg & 2 kg ma van dung chinh xac +-1 mm.
                 */
                float leg2_odom_ref = TRACK_LEG2_DIST_M
                                    + ((load_class_kg == 2U) ? TRACK_LEG2_2KG_ODOM_COMP_M : 0.0f);
                float d_rem = (leg2_odom_ref - ROBOT_SENSOR_OFFSET_M) - drv.s_m;
                float v_prof = outer_calc_decel_velocity(d_rem, smc.p.v_req, ROBOT_V_MIN_MPS,
                                                         TRACK_DECEL_WINDOW_M, TRACK_DECEL_MARGIN_M);
                float v_junc = outer_calc_y_junction_speed(drv.s_m, smc.p.v_req);
                v_cmd_out        = fminf(v_prof, v_junc);
                smc_track_active = true;

                if (eol_triggered) {
                    float d_brake = outer_estimate_brake_lead_m(drv.v_mps, load_measured_g * 0.001f);
                    s_eol_start_m = drv.s_m;
                    d_goal_adv_m  = ROBOT_SENSOR_OFFSET_M - GOAL_EOL_PSF_OFFSET_M
                                  + smc.p.goal_trim_m - d_brake;
                    if (d_goal_adv_m < 0.005f) d_goal_adv_m = 0.005f;

                    mstate        = MISSION_GOAL_AXLE_ALIGN;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGI(TAG, "EOL detected at s=%.3f m -> creeping %.1f mm straight to align drive axle",
                             drv.s_m, d_goal_adv_m * 1000.0f);
                } else if (drv.s_m >= TRACK_GOAL_WIN_MAX_M) {
                    drive_service_stop_brake();
                    goal_stop_s_m = drv.s_m;
                    goal_creep_mm = 0.0f;
                    mstate        = MISSION_STOPPED_GOAL;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGW(TAG, "Goal failsafe odometry reached (s=%.3f m) -> Short Brake", drv.s_m);
                }
                break;
            }

            case MISSION_GOAL_AXLE_ALIGN: {
                /* Khoa lai thang (w_cmd = 0) va bo cham dua TRUNG DIEM TRUC BANH vao dung vach dich */
                v_cmd_out        = ROBOT_V_MIN_MPS;
                smc_track_active = false;
                float crept_m    = drv.s_m - s_eol_start_m;

                if (crept_m >= d_goal_adv_m || state_timer_s >= 1.5f) {
                    drive_service_stop_brake();
                    goal_stop_s_m = drv.s_m;
                    goal_creep_mm = crept_m * 1000.0f;
                    mstate        = MISSION_STOPPED_GOAL;
                    state_timer_s = 0.0f;
                    s_cur_state   = mstate;
                    ESP_LOGI(TAG, "GOAL REACHED! Axle s=%.3f m (creep=%.1f mm, odom err=%+.1f mm) -> holding 2.0 s",
                             goal_stop_s_m, goal_creep_mm, (goal_stop_s_m - TRACK_LEG2_DIST_M) * 1000.0f);
                } else {
                    drive_service_set_chassis_vel(ROBOT_V_MIN_MPS, 0.0f);
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

        /* 5. Cap nhat Bo quan sat Luenberger (100 Hz) & Bo lap lich SMC thich nghi.
         *    Su dung van toc thuc sau S-curve (v_eff_ctrl) de SMC khong van lai manh luc khoi hanh! */
        float v_eff_ctrl = drv.scurve_active ? fminf(v_cmd_out, drv.v_cmd_eff_mps) : v_cmd_out;

        outer_estimator_step_observer_100hz(&est, drv.v_mps, drv.w_rads,
                                            v_eff_ctrl, smc_track_active,
                                            line.line_detected, OBSERVER_LOOP_TS_S);

        bool smc_updated = outer_smc_step_adaptive(&smc,
                                                   est.e2_filt, est.e3_hat, est.c_hat,
                                                   v_eff_ctrl, seg_elapsed_s,
                                                   drv.v_a_ref_mps, drv.v_a_mps,
                                                   drv.v_b_ref_mps, drv.v_b_mps,
                                                   smc_track_active, line.line_detected,
                                                   OBSERVER_LOOP_TS_S);
        if (smc_updated) {
            sum_dt_smc_ms += (double)(smc.last_dt_smc_s * 1000.0f);
            smc_update_cnt++;
        }

        /* Gui lenh (v_cmd, w_cmd) xuong vong trong 200 Hz (Core 1) */
        if (smc_track_active) {
            drive_service_set_chassis_vel(v_cmd_out, smc.w_cmd);
        }

        /* 6. Tinh KPI sai so bam line (sau 0.6s khoi hanh) */
        if (smc_track_active && seg_elapsed_s > 0.60f && line.line_detected) {
            float e2_mm = fabsf(est.e2_filt) * 1000.0f;
            if (e2_mm > max_abs_e2_mm) max_abs_e2_mm = e2_mm;
            sum_sq_e2_mm += (double)(e2_mm * e2_mm);
            rms_e2_cnt++;
        }

        /* 7. Ghi log SRAM (He B) hoac Live Stream UDP (He A) */
        bool should_emit = false;
        if (job->cfg.enable_sram_log) {
            /* Trong luc dung cho nguoi dat hang: giam toc do log xuong 5 Hz de tiet kiem SRAM */
            if (mstate == MISSION_WAIT_LOAD) {
                should_emit = ((wait_log_div++ % SYSTEM_B_WAIT_LOG_DIV) == 0U);
            } else {
                should_emit = true;
            }
        } else if (job->cfg.live_cb != NULL) {
            should_emit = ((tick_100hz % SYSTEM_A_LIVE_DIV) == 0U);
        }

        if (should_emit) {
            uint32_t t_ms = (uint32_t)((esp_timer_get_time() - run_start_us) / 1000LL);
            robot_log_sample_t sample;
            pack_log_sample(&sample, t_ms, &est, &smc, &drv, &line,
                            mstate, smc_updated, marker_seen, load_class_kg);

            if (job->cfg.enable_sram_log) {
                data_logger_push(&sample);
            }
            if (job->cfg.live_cb != NULL) {
                job->cfg.live_cb(&sample);
            }
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

    /* Tong ket KPI */
    uint32_t dur_ms = (uint32_t)((esp_timer_get_time() - run_start_us) / 1000LL);
    memset(&job->summary, 0, sizeof(job->summary));
    job->summary.run_mode        = (uint8_t)job->cfg.mode;
    job->summary.sensor_profile  = (uint8_t)job->cfg.sensor_profile;
    job->summary.stop_reason     = (uint8_t)stop_reason;
    job->summary.load_class_kg   = load_class_kg;
    job->summary.sample_count    = job->cfg.enable_sram_log ? data_logger_count() : tick_100hz;
    job->summary.run_duration_ms = dur_ms;
    job->summary.load_measured_g = load_measured_g;
    job->summary.max_abs_e2_mm   = max_abs_e2_mm;
    job->summary.rms_e2_mm       = (rms_e2_cnt > 0) ? (float)sqrt(sum_sq_e2_mm / (double)rms_e2_cnt) : 0.0f;
    job->summary.station_stop_s_m= station_stop_s_m;
    job->summary.goal_stop_s_m   = goal_stop_s_m;
    job->summary.goal_creep_mm   = goal_creep_mm;
    job->summary.avg_dt_smc_ms   = (smc_update_cnt > 0) ? (float)(sum_dt_smc_ms / (double)smc_update_cnt) : 0.0f;
    job->summary.y_gn_calls      = y_gn_calls;

    data_logger_set_summary(&job->summary);
    job->stop_reason = stop_reason;

    ESP_LOGI(TAG, "Mission END | reason=%d dur=%lu ms | max|e2|=%.2f mm RMS=%.2f mm | avg_dt_smc=%.1f ms | log=%lu",
             (int)stop_reason, (unsigned long)dur_ms,
             job->summary.max_abs_e2_mm, job->summary.rms_e2_mm,
             job->summary.avg_dt_smc_ms, (unsigned long)job->summary.sample_count);

    s_cur_state = MISSION_IDLE;
    s_running   = false;
    xTaskNotifyGive(job->caller_task);
    vTaskDelete(NULL);
}

robot_stop_reason_t mission_ctrl_run_blocking(const mission_run_cfg_t *cfg,
                                              robot_run_summary_t *out_summary)
{
    if (cfg == NULL) return STOP_REASON_FAULT;
    if (!s_inited) mission_ctrl_init();

    s_stop_req  = false;
    s_running   = true;
    s_cur_state = MISSION_IDLE;

    /* Pha dem nguoc truoc khi chay (max(3s, t_switch) cho 7S / 3s cho 5S) */
    if (cfg->countdown_rem_ms > 0) {
        ESP_LOGI(TAG, "Countdown %lu ms before launch (press BOOT or send STOP to abort)...",
                 (unsigned long)cfg->countdown_rem_ms);
        usr_led_start_blink(150);
        uint32_t waited = 0;
        while (waited < cfg->countdown_rem_ms) {
            if (s_stop_req || gpio_get_level(PIN_BOOT_BUTTON) == 0) {
                usr_led_set(false);
                s_running = false;
                ESP_LOGW(TAG, "Launch aborted during countdown");
                /* Doi nha nut BOOT */
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

    /* Cho den khi outer_ctrl_task ket thuc */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (out_summary != NULL) {
        *out_summary = job.summary;
    }
    return job.stop_reason;
}
