#pragma once
#include <stdint.h>

typedef enum {
    ENC_CH_A = 0,
    ENC_CH_B = 1
} enc_ch_t;

/* GỌI TRONG TASK GHIM TRÊN CORE 1 để ISR gắn trực tiếp lên Core 1 */
void encoder_init_on_core1(void);

/* Xóa bộ đếm & trạng thái M/T trước mỗi lượt đo */
void encoder_reset(void);

/* Đọc tốc độ RPM theo phương pháp M/T và số xung delta trong chu kỳ 5ms */
float encoder_sample_rpm(enc_ch_t ch, int64_t now_us, int16_t *out_delta);