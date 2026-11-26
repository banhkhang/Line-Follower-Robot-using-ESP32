#ifndef SENSOR_SERVICE_H
#define SENSOR_SERVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"
#include "tcrt5000.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int   min[LINE_SENSOR_MAX_COUNT];     /* Lowest ambient reading seen (brightest light) */
    int   max[LINE_SENSOR_MAX_COUNT];     /* Highest ambient reading seen (darkest shadow) */
    float avg[LINE_SENSOR_MAX_COUNT];     /* Running average ambient ADC across samples */
    int   current[LINE_SENSOR_MAX_COUNT]; /* Latest single-shot ambient reading (val_off) */
} line_sensor_amb_t;

/*
 * All arrays are ALWAYS indexed by physical sensor slot 0..6 == S1..S7:
 *   S1 = Leftmost (-27 mm PCB / +27 mm SMC) ... S7 = Rightmost (+27 mm PCB / -27 mm SMC).
 * In TCRT_PROFILE_5S, slots [0] and [6] have raw = -1, norm = 0.0f, i_eff = 0.0f.
 */
typedef struct {
    int               raw[LINE_SENSOR_MAX_COUNT];   /* Ambient-compensated net ADC */
    float             norm[LINE_SENSOR_MAX_COUNT];  /* Normalized 0.0 (white) .. 100.0 (black) */
    float             i_eff[LINE_SENSOR_MAX_COUNT]; /* SMC reflectance [0..1] after noise floor */
    line_sensor_amb_t amb;                          /* Ambient light statistics */
    float             total_i;                      /* Sum of i_eff across active sensors */
    uint8_t           n_active;                     /* Count of sensors with i_eff > 0.15 */
    float             position_mm;                  /* PCB frame error [mm] (Left < 0, Right > 0) */
    float             e2_smc_m;                     /* SMC ISO frame error [m] (Left > 0, Right < 0) */
    bool              line_detected;                /* True if total_i >= OUTER_LINE_TOTAL_I_MIN */
    tcrt_profile_t    profile;                      /* Active sensor profile (5 or 7) */
} line_sensor_reading_t;

/* Initialize sensor service with chosen runtime profile (loads saved NVS calib if present) */
esp_err_t      sensor_service_init(tcrt_profile_t initial_profile);
void           sensor_service_deinit(void);
bool           sensor_service_is_enabled(void);

/* Dynamically switch between 5-sensor (Wi-Fi ON) and 7-sensor (Wi-Fi OFF) profiles */
esp_err_t      sensor_service_set_profile(tcrt_profile_t profile);
tcrt_profile_t sensor_service_get_profile(void);

/* Load / save 7-sensor R_white & R_black calibration arrays in NVS flash */
esp_err_t      sensor_service_load_calib_nvs(void);
esp_err_t      sensor_service_save_calib_nvs(void);
bool           sensor_service_has_valid_calib(void);

/*
 * Interactive 7-sensor sweep calibration (Wi-Fi MUST be OFF before calling!).
 *  - Switches to TCRT_PROFILE_7S.
 *  - Turns LED1 SOLID ON during sweeping; wave sensor bar across white floor & black line.
 *  - Press BOOT button to finish -> commits 10%-guarded R_white/R_black, saves to NVS,
 *    turns LED1 OFF.
 */
esp_err_t      sensor_service_calibrate_sweep(void);

/* Perform one ambient-compensated oversampled read and compute norm, i_eff, position_mm, e2_smc_m */
esp_err_t      sensor_service_read(line_sensor_reading_t *out_reading);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_SERVICE_H
