#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Runtime sensor profile selection:
 *  - TCRT_PROFILE_5S: Symmetric 5 center sensors (S2..S6, ADC1 only).
 *                     100% compatible with active Wi-Fi (System A / Mode 1).
 *  - TCRT_PROFILE_7S: Full 7-sensor array (S1..S7, ADC1 + ADC2 on GPIO25).
 *                     Wi-Fi MUST be turned OFF before switching to this profile
 *                     (System B / Standalone 7S / Calibration).
 *
 * IMPORTANT INDEXING CONVENTION:
 *  All output arrays (raw[7], net[7], amb[7]) are ALWAYS indexed by physical
 *  PCB sensor slot 0..6 corresponding to S1..S7 (S1 = Left -27 mm, S7 = Right +27 mm).
 *  In TCRT_PROFILE_5S, slots [0] (S1) and [6] (S7) are set to -1.
 */
typedef enum {
    TCRT_PROFILE_5S = 5,
    TCRT_PROFILE_7S = 7
} tcrt_profile_t;

esp_err_t      tcrt5000_init(tcrt_profile_t profile);
esp_err_t      tcrt5000_deinit(void);
esp_err_t      tcrt5000_set_profile(tcrt_profile_t profile);
tcrt_profile_t tcrt5000_get_profile(void);
bool           tcrt5000_is_initialized(void);

void           tcrt5000_set_emitters(bool enable);

/* Read single-shot raw 12-bit ADC values (0..4095) into physical slots [0..6]. */
esp_err_t      tcrt5000_read_raw(int raw_values[LINE_SENSOR_MAX_COUNT]);

/*
 * Ambient-light-compensated read with optional back-to-back ADC oversampling:
 *   1) IR LEDs ON -> wait 80 us -> average `oversample` ADC reads per active channel
 *   2) IR LEDs OFF -> wait 80 us -> average `oversample` ADC reads per active channel
 *   3) net[i] = max(0, val_off[i] - val_on[i])
 * Uses polled ADC oneshot (zero extra interrupts) to reject ADC thermal noise
 * while keeping CPU load low (~0.7-0.9 ms total for oversample=2).
 */
esp_err_t      tcrt5000_read_compensated_avg(int net_values[LINE_SENSOR_MAX_COUNT],
                                             int amb_values[LINE_SENSOR_MAX_COUNT],
                                             uint8_t oversample);

esp_err_t      tcrt5000_read_compensated(int net_values[LINE_SENSOR_MAX_COUNT],
                                         int amb_values[LINE_SENSOR_MAX_COUNT]);

#ifdef __cplusplus
}
#endif
