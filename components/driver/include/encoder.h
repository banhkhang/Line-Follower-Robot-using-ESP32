#pragma once

#include <stdint.h>

/*
 * ============================================================================
 * encoder.h - Doc van toc M/T 1x & Odometry tu Encoder Hall tren Core 1
 * ============================================================================
 *
 * NGUYEN LY DO VAN TOC KET HOP XUNG - THOI GIAN (M/T 1x):
 *  - Neu dem xung co dinh trong Ts = 5 ms (Method M), o 0.3 m/s sai so luong
 *    tu hoa +/-1 xung len toi >10%. Phuong phap M/T do khoang thoi gian thuc
 *    dt_us (do phan giai 1 us) giua canh len cuoi cung cua chu ky hien tai va
 *    canh len cuoi cung cua chu ky truoc do:
 *        n [RPM] = (60e6 * delta_N) / (N_1x * dt_us)
 *    giup giam sai so luong tu hoa xuong ~0.024% ma khong can bo loc thong thap
 *    gay tre pha.
 *  - Tai sao dung che do 1x (chi ngat canh len kenh A, N_1x = 330 xung/vong)
 *    thay vi 4x (1320 xung/vong)?
 *      1) Loai bo nhieu chu ky 2*f_Hall do lech pha 90 deg va duty-cycle != 50%
 *         giua hai cam bien Hall A/B (nhieu nen giam tu 1.8-6.4 mm/s xuong ~0.8 mm/s).
 *      2) Giam 4 lan tan suat ngat (con ~2262 ngat/s cho 2 kenh o 0.7 m/s), triet
 *         tieu hien tuong sot xung gay gai van toc.
 *      3) Chieu quay duoc doc tuc thoi trong IRAM ISR qua thanh ghi GPIO_IN_REG
 *         cua kenh B (khong can gan ngat len kenh B).
 *
 * BON CO CHE BAO VE TRONG DRIVER:
 *  1) Loc nhieu EMI (< 80 us): loai xung gia cau H (o 350 RPM, T_min = 519.5 us).
 *  2) Dong bo moc canh dau (dt > 50 ms): khi moi khoi dong tu dung yen, lay canh
 *     dau lam moc thoi gian va xuat 0 RPM de khong tao gai.
 *  3) Nguong gioi han ve 0 (timeout 25 ms): khi ham/dung, gioi han toc do theo
 *     duong bao 1/(now - last_edge), va ep ve 0 RPM sau 25 ms (< 7.27 RPM).
 *  4) Bo chan gai mat xung don le (|dn| > 30 RPM trong 5 ms): giu mau truoc
 *     dung 1 chu ky neu nhay vat ly bat kha thi (> 20.4 m/s^2), chap nhan o
 *     nhip ke tiep neu thay doi la that -> khong gay tre pha dap ung thuc.
 * ============================================================================
 */

typedef enum {
    ENC_CH_A = 0,                   /* Encoder A - banh TRAI */
    ENC_CH_B = 1,                   /* Encoder B - banh PHAI */
    ENC_CH_LEFT  = ENC_CH_A,
    ENC_CH_RIGHT = ENC_CH_B
} enc_ch_t;

/*
 * Khoi tao GPIO va cai dat ngat canh len IRAM ISR cho ca 2 encoder.
 * QUAN TRONG: PHAI GOI TRONG TASK GHIM TREN CORE 1 de ISR phuc vu doc quyen
 * tren Core 1, tranh tranh chap voi Wi-Fi / Telemetry / HX711 tren Core 0.
 */
void encoder_init_on_core1(void);

/* Xoa toan bo bo dem xung va trang thai M/T ve 0 (dung khi khoi dong lai). */
void encoder_reset(void);

/*
 * Lay mau toc do quay truc banh xe [RPM] tai thoi diem now_us [us] theo M/T 1x.
 *  - ch        : ENC_CH_A (trai) hoac ENC_CH_B (phai)
 *  - now_us    : dau thoi gian hien tai tu esp_timer_get_time() [us]
 *  - out_delta : (tuy chon, co the NULL) tra ve so xung tich luy trong chu ky vua qua
 */
float encoder_sample_rpm(enc_ch_t ch, int64_t now_us, int16_t *out_delta);

/*
 * Doc tong so xung 1x tich luy co dau ke tu lan encoder_reset() gan nhat.
 * Dung de tinh quang duong odometry (s_enc) chinh xac tung 0.62 mm ma khong bi
 * troi sai so tich phan so, va cho phep reset moc quang duong ma khong lam nhieu M/T.
 */
int32_t encoder_get_count(enc_ch_t ch);
