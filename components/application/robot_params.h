#pragma once
/*
 * ============================================================================
 * robot_params.h - Tap hop tap trung thong so Dong hoc & Dong luc hoc Robot
 * ============================================================================
 *
 * NGUYEN TAC QUY CHUAN (Single Source of Truth):
 *  - Moi thong so hinh hoc, co-dien, mo hinh ham truyen nhan dang, luat tham so
 *    vong trong 2-DOF PI va tham so vong ngoai (Alpha-Beta, Gauss-Newton,
 *    Luenberger Observer, Integral SMC, quy hoach van toc & sa ban) duoc dat
 *    duy nhat tai day.
 *  - Khi co xung dot giua thiet lap thuc nghiem phan cung that va file mo phong
 *    MATLAB `SMC_controller14.m`, TA LAY THONG SO THUC TE TAI DAY LAM CHUAN:
 *      1) Khoang cach tam 2 banh chu dong thuc te: B = 0.180 m (180 mm).
 *      2) Khoang cach truc banh chu dong (phia truoc) -> day cam bien: d1 = 0.055 m.
 *      3) Tan so vong trong: fs = 200 Hz (Ts = 5 ms), 2-DOF PI theo don vi [m/s]
 *         co luat thich nghi theo tai trong m_load tren mat bat sa ban (w_gc = 30 rad/s).
 *      4) Van toc dai danh dinh: v_req = 0.50 m/s (thay vi 0.40 m/s trong SMC_controller14.m).
 *      5) Diem tham chieu dung tai Tram tai hang & Vach ket thuc: TRUNG DIEM
 *         TRUC BANH CHU DONG (cach day cam bien d1 = 55 mm ve phia sau).
 *      6) Quy uoc kenh dong co (XAC NHAN THUC TE TREN XE):
 *           Motor A (PWMA1/2, IO12/13) = banh PHAI (phia cam bien S7)
 *           Motor B (PWMB1/2, IO16/15) = banh TRAI (phia cam bien S1)
 *         Quy uoc he truc SMC (he tay phai ISO): +X tien, +Y sang TRAI, +w re TRAI (CCW).
 *         (Tren PCB, S1 nam ben TRAI ung voi -27 mm trong sensor_service -> tang
 *          vong ngoai dao dau sang +27 mm de dung quy uoc SMC: e2 > 0 -> w > 0 re trai.)
 * ============================================================================
 */
/* -------------------------------------------------------------------------- */
/* 1. THONG SO HINH HOC & DONG HOC KHUNG XE (Kinematics & Geometry)           */
/* -------------------------------------------------------------------------- */

/* Banh xe chu dong V2 (dat o phia truoc): duong kinh D = 65 mm, ban kinh R = 32.5 mm */
#define ROBOT_WHEEL_DIAMETER_M      0.065f
#define ROBOT_WHEEL_RADIUS_M        0.0325f
#define ROBOT_WHEEL_CIRCUM_M        0.20420352f     /* C = pi * D [m] */
/* Quy doi toc do quay truc banh xe [RPM] <-> van toc dai [m/s] */
#define ROBOT_RPM_TO_MPS            (ROBOT_WHEEL_CIRCUM_M / 60.0f)
#define ROBOT_MPS_TO_RPM            (60.0f / ROBOT_WHEEL_CIRCUM_M)
/*
 * Khoang cach tam hai banh chu dong (Track width B) [m] = 180 mm (thuc te).
 *
 * Quan he dong hoc vi sai (tam quy chieu la TRUNG DIEM TRUC BANH CHU DONG,
 * he truc tay phai ISO: +X tien, +Y sang trai, omega > 0 la re trai / CCW):
 *   Dong hoc nghich (Motor A = banh PHAI phia S7, Motor B = banh TRAI phia S1):
 *     v_right_ref (Motor A) = v_cmd + (B / 2) * w_cmd
 *     v_left_ref  (Motor B) = v_cmd - (B / 2) * w_cmd
 *   Dong hoc thuan (tinh odometry tra ve cho Observer/SMC):
 *     v_act = 0.5 * (v_right + v_left) = 0.5 * (v_A + v_B)
 *     w_act = (v_right - v_left) / B   = (v_A - v_B) / B
 */
#define ROBOT_TRACK_WIDTH_M         0.180f
#define ROBOT_HALF_TRACK_M          (ROBOT_TRACK_WIDTH_M * 0.5f)
/* Kich thuoc tong the khung xe, banh tu do sau, vi tri trong tam (CoG) & cam bien */
#define ROBOT_BODY_LENGTH_M         0.240f          /* Chieu dai than xe thuc te [m] (240 mm) */
#define ROBOT_BODY_WIDTH_M          0.136f          /* Chieu rong than xe [m] */
#define ROBOT_SENSOR_OFFSET_M       0.055f          /* d1: truc banh truoc -> day cam bien [m] */
#define ROBOT_WHEELBASE_CASTER_M    0.152f          /* Khoang cach truc banh chu dong -> banh tu do sau [m] */
#define ROBOT_PAYLOAD_X_OFFSET_M    0.0825f         /* Tam dat tai cach truc banh truoc ve phia sau [m] */
#define ROBOT_PAYLOAD_Z_HEIGHT_M    0.048f          /* Do cao day khoi hang so voi mat san [m] */
#define ROBOT_COG_OFFSET_0KG_M      0.037f          /* Trong tam cach truc banh truoc khi 0 kg [m] */
#define ROBOT_COG_OFFSET_1KG_M      0.049f          /* Trong tam cach truc banh truoc khi 1 kg [m] */
#define ROBOT_COG_OFFSET_2KG_M      0.057f          /* Trong tam cach truc banh truoc khi 2 kg [m] */

#define ROBOT_LINE_WIDTH_M          0.026f          /* Be rong vach line den [m] = 26 mm */
#define ROBOT_CROSSBAR_WIDTH_M      0.026f          /* Be rong doc X cua vach cat ngang tram tai [m] */
#define ROBOT_CROSSBAR_LEN_M        0.050f          /* Chieu dai ngang Y cua vach cat ngang tram tai [m] */
#define ROBOT_LINE_SENSOR_COUNT     7               /* 7 mat hong ngoai TCRT5000 */
#define ROBOT_LINE_SENSOR_PITCH_M   0.009f          /* Buoc mat cam bien = 9 mm ([-27..+27] mm) */
/* -------------------------------------------------------------------------- */
/* 2. THONG SO KHOI LUONG & CO CAU CHAP HANH (Mass & Actuator)                */
/* -------------------------------------------------------------------------- */
/* Khoi luong xe khong tai thuc te va dai tai trong hang hoa (Loadcell HX711) */
#define ROBOT_MASS_EMPTY_KG         1.30f           /* m_xe [kg] */
#define ROBOT_PAYLOAD_MIN_KG        0.00f           /* m_load min [kg] */
#define ROBOT_PAYLOAD_MAX_KG        2.00f           /* m_load dinh muc max [kg] */
#define ROBOT_PAYLOAD_CLAMP_MAX_KG  2.50f           /* Nguong bao hoa an toan cho luat tham so [kg] */
/* Dong co DC giam toc JGB37-520 12V 333 RPM, hop so 30:1, Hall 2 kenh 11 PPR */
#define MOTOR_GEAR_RATIO            30.0f
#define ENC_ROTOR_PPR               11.0f           /* So xung/vong rotor tren 1 kenh */
#define ENC_PPR_1X                  (ENC_ROTOR_PPR * MOTOR_GEAR_RATIO)      /* 330 xung/vong truc ra (1x) */
#define ENC_CPR_4X                  (ENC_PPR_1X * 4.0f)                     /* 1320 xung/vong truc ra (4x) */
#define ENC_DIST_PER_PULSE_1X_M     (ROBOT_WHEEL_CIRCUM_M / ENC_PPR_1X)     /* ~0.6188 mm/xung */
/* -------------------------------------------------------------------------- */
/* 3. KIEN TRUC DA TAN SO (Multi-Rate Control Timing)                         */
/* -------------------------------------------------------------------------- */
/*
 * Kien truc da tan so dong bo pha tu 1 nguon ngat duy nhat (GPTimer 200 Hz, Core 1):
 *  - Tang 1 (200 Hz, Ts = 5 ms, Core 1): Vong trong 2-DOF PI + Encoder M/T 1x.
 *  - Tang 2 (200 Hz, Ts = 5 ms, Core 0): Doc TCRT5000 (oversample 2x, 0 ngat phu)
 *            + Bo loc Alpha-Beta + phat hien vach ngang / het line / watchdog an toan.
 *  - Tang 3 (100 Hz, Ts = 10 ms, divider = 2): Tach nhanh Gauss-Newton (vung chu Y)
 *            + Bo quan sat Luenberger thich nghi + May trang thai nhiem vu + Ghi log.
 *  - Tang 4 (Thich nghi trong [50 ms .. 180 ms]): Bo dieu khien truot Integral SMC
 *            cap nhat ngay khi vong trong da xac lap + le an toan (co chan duoi 50 ms
 *            va chan tren < 200 ms de tranh tre pha khi vao cua).
 */

 #define INNER_LOOP_FS_HZ            200U
#define INNER_LOOP_TS_US            5000U
#define INNER_LOOP_TS_S             0.005f
#define SENSOR_LOOP_FS_HZ           200U
#define SENSOR_LOOP_TS_S            0.005f
#define OBSERVER_LOOP_FS_HZ         100U
#define OBSERVER_LOOP_TS_S          0.010f
#define OBSERVER_LOOP_DIVIDER       (INNER_LOOP_FS_HZ / OBSERVER_LOOP_FS_HZ)  /* 2 */
#define OUTER_SMC_FS_HZ             20U             /* Tan so danh dinh toi da cua SMC (Ts_min = 50 ms) */
#define OUTER_SMC_TS_S              0.050f
#define OUTER_SMC_DIVIDER           (INNER_LOOP_FS_HZ / OUTER_SMC_FS_HZ)      /* 10 */
/* -------------------------------------------------------------------------- */
/* 4. THONG SO DONG LUC HOC DONG CO DA NHAN DANG (Identified Plant Models)    */
/* -------------------------------------------------------------------------- */
#define PLANT_BENCH_KDC_A           0.01175f        /* [(m/s) / %PWM] */
#define PLANT_BENCH_TAU_A_S         0.0762f         /* [s] */
#define PLANT_BENCH_UDZ_A_PCT       2.40f           /* [%PWM] */
#define PLANT_BENCH_KDC_B           0.01225f        /* [(m/s) / %PWM] */
#define PLANT_BENCH_TAU_B_S         0.0952f         /* [s] */
#define PLANT_BENCH_UDZ_B_PCT       1.68f           /* [%PWM] */
#define PLANT_MAT_TAU0_A_S          0.1128f         /* [s] tai 0 kg hang (1.3 kg xe) */
#define PLANT_MAT_TAU_SLOPE_A       0.0262f         /* [s/kg] */
#define PLANT_MAT_TAU0_B_S          0.1328f         /* [s] tai 0 kg hang (1.3 kg xe) */
#define PLANT_MAT_TAU_SLOPE_B       0.0324f         /* [s/kg] */
/* -------------------------------------------------------------------------- */
/* 5. LUAT THAM SO BO DIEU KHIEN 2-DOF PI VONG TRONG (Gain Scheduling)        */
/* -------------------------------------------------------------------------- */
#define VEL_PI_MAT_A_KP0            291.0f          /* [%PWM / (m/s)] */
#define VEL_PI_MAT_A_KP_SLOPE       67.6f           /* [%PWM / ((m/s)*kg)] */
#define VEL_PI_MAT_A_KI             2064.0f         /* [%PWM / m] */
#define VEL_PI_MAT_A_KFF            86.0f           /* [%PWM / (m/s)] */
#define VEL_PI_MAT_A_UDZ0           4.51f           /* [%PWM] */
#define VEL_PI_MAT_A_UDZ_SLOPE      2.78f           /* [%PWM / kg] */

#define VEL_PI_MAT_B_KP0            326.7f          /* [%PWM / (m/s)] */
#define VEL_PI_MAT_B_KP_SLOPE       79.7f           /* [%PWM / ((m/s)*kg)] */
#define VEL_PI_MAT_B_KI             1968.0f         /* [%PWM / m] */
#define VEL_PI_MAT_B_KFF            82.0f           /* [%PWM / (m/s)] */
#define VEL_PI_MAT_B_UDZ0           7.32f           /* [%PWM] */
#define VEL_PI_MAT_B_UDZ_SLOPE      5.21f           /* [%PWM / kg] */
#define VEL_PI_MAT_EBAND0           0.020f          /* [m/s] */
#define VEL_PI_MAT_EBAND_SLOPE      0.005f          /* [(m/s) / kg] */
/* Bo tham so du phong tren ga do (w_gc = 35 rad/s, PM ~ 70..74 deg - Bang 6.3) */
#define VEL_PI_BENCH_A_KP           227.0f
#define VEL_PI_BENCH_A_KI           2978.0f
#define VEL_PI_BENCH_A_KFF          85.51f
#define VEL_PI_BENCH_A_UDZ          3.10f
#define VEL_PI_BENCH_B_KP           272.0f
#define VEL_PI_BENCH_B_KI           2856.0f
#define VEL_PI_BENCH_B_KFF          82.40f
#define VEL_PI_BENCH_B_UDZ          1.71f
#define VEL_PI_BENCH_EBAND          0.035f
#define VEL_PI_U_MAX_PCT            100.0f          /* Gioi han PWM toi da [%] */
/* Nguong phi tuyen ben trong bo dieu khien 2-DOF PI */
#define VEL_CTRL_V_REF_EPS_MPS      0.005f          /* |v_ref| <= 5 mm/s -> Short Brake 0% */
#define VEL_CTRL_REV_LOCK_THR_MPS   0.050f          /* v_ref > 50 mm/s -> khoa u_min = 0% (Anti-Plugging) */

/* -------------------------------------------------------------------------- */
/* 6. GIOI HAN VAN HANH, S-CURVE & QUANG DUONG PHANH (Operating Limits)       */
/* -------------------------------------------------------------------------- */
/* Chi tieu van hanh cua vong trong (Bang 6.1) */
#define ROBOT_V_NOM_MPS             0.50f           /* Van toc dai danh dinh v_req [m/s] */
#define ROBOT_V_MIN_MPS             0.10f           /* Van toc di chuyen toi thieu theo de bai [m/s] */
#define ROBOT_V_WHEEL_MIN_OP_MPS    0.30f           /* Can duoi vung cua (R_min = 0.5 m) [m/s] */
#define ROBOT_V_WHEEL_MAX_OP_MPS    0.70f           /* Can tren vung cua (R_min = 0.5 m) [m/s] */
#define ROBOT_V_WHEEL_ABS_MAX_MPS   0.85f           /* Gioi han bao ve van toc dat moi banh [m/s] */
#define ROBOT_W_MAX_RADS            4.50f           /* Gioi han van toc goc dat cua vong ngoai SMC [rad/s] */
#define ROBOT_A_MAX_MPS2            1.00f           /* Gia toc cuc dai chong truot banh [m/s^2] */
#define ROBOT_MIN_TURN_RADIUS_M     0.50f           /* Ban kinh cua nho nhat tren sa ban [m] */
/* Quy dao khoi dong muot S-curve bac 3 (Muc 6.1.1): 0 -> 0.5 m/s trong 0.80 s */
#define ROBOT_SCURVE_DEFAULT_T_S    0.80f
/* Nguong nhan dien xe da dung han (su dung cho may trang thai tram tai hang / ve dich) */
#define ROBOT_STOP_RPM_THR          0.50f           /* [RPM] ~ 1.7 mm/s */
/* Quang duong phanh Short Brake thuc do tu 0.7 -> 0 m/s tren bat (Bang 6.6) */
#define ROBOT_STOP_DIST_REF_V_MPS   0.70f           /* Van toc thu nghiem phanh Bang 6.6 [m/s] */
#define ROBOT_STOP_DIST_0KG_M       0.064f          /* TB A/B: 57.9 / 69.8 mm */
#define ROBOT_STOP_DIST_1KG_M       0.071f          /* TB A/B: 65.0 / 76.9 mm */
#define ROBOT_STOP_DIST_2KG_M       0.078f          /* TB A/B: 72.0 / 84.0 mm */
/* -------------------------------------------------------------------------- */
/* 7. THAM SO VONG NGOAI: ALPHA-BETA, GAUSS-NEWTON, LUENBERGER & SMC          */
/*    (Tham chieu SMC_controller14.m & Doria-Cerezo et al., ECC 2019)         */
/* -------------------------------------------------------------------------- */

/* 7.1 Bo loc tien xu ly Alpha-Beta cho sai so ngang e2 */
#define OUTER_AB_WN_RADS            75.0f           /* Tan so tu nhien bo loc [rad/s] */
#define OUTER_AB_ZETA               0.95f           /* He so tat dan toi uu */
/* 7.2 Bo tach nhanh chu Y Gauss-Newton (100 Hz) */
#define OUTER_OPT_DELTA_M           0.0058f         /* Do rong chuyen tiep quang hoc tanh() [m] */
#define OUTER_GN_GATE_HALF_W_M      0.021f          /* Cua so gating |y_sens - e_pred| <= 21 mm */
#define OUTER_GN_MAX_STEP_M         0.008f          /* Gioi han buoc cap nhat moi vong lap [m] */
#define OUTER_GN_ITERS              3               /* So vong lap Gauss-Newton */
#define OUTER_GN_NOISE_OFFSET       0.12f           /* Bu nen quang hoc trong mo hinh template */
#define OUTER_GN_MIN_ACTIVE         0.06f           /* Nguong cat duoi cua T0 */
#define OUTER_SINGLE_LINE_TOT_7S    3.05f           /* Tong cuong do 1 vach 26 mm thuc do (profile 7 mat; 1 line ~2.5-2.9) */
#define OUTER_SINGLE_LINE_TOT_5S    2.95f           /* Tong cuong do 1 vach 26 mm thuc do (profile 5 mat; 1 line ~2.4-2.85) */
#define OUTER_GN_BLEND_SPAN         0.20f           /* Khoang tron muot centroid -> Gauss-Newton */
#define OUTER_LINE_TOTAL_I_MIN      0.25f           /* Nguong tong I_eff toi thieu de coi la thay line */
/* 7.3 Bo quan sat Luenberger thich nghi (100 Hz, Ts_in = 0.010 s) */
#define OUTER_OBS_TAU_C_S           0.350f          /* Tach bang thong do cong (<0.45 Hz) khoi nhip lai (~1.5 Hz) -> tang giam chan 3.4x */
#define OUTER_OBS_V_MIN_DENOM       0.30f           /* Mau so van toc toi thieu on dinh do loi [m/s] */
#define OUTER_OBS_V_ACTIVE_THR      0.04f           /* Nguong van toc kich hoat quan sat [m/s] */
#define OUTER_OBS_P1_SLOW           0.87f           /* Cuc cham (giam tre pha goc e3_hat, van loc nhieu tot) */
#define OUTER_OBS_P2_SLOW           0.91f
#define OUTER_OBS_P1_FAST           0.75f           /* Cuc nhanh (bat nhanh goc e3 khi chuyen tiep cua) */
#define OUTER_OBS_P2_FAST           0.80f
#define OUTER_OBS_ERR_BLEND_LO_M    0.00045f        /* Nguong duoi residual chuyen sang cuc nhanh [m] */
#define OUTER_OBS_ERR_BLEND_SPAN_M  0.00090f        /* Khoang chuyen tiep muot slow -> fast [m] */
#define OUTER_OBS_E3_CLAMP_RAD      1.05f           /* Gioi han an toan |e3_hat| <= ~60 deg */
#define OUTER_OBS_C_MAX_RADM        2.20f           /* Gioi han vat ly do cong |c_hat| <= 2.2 rad/m (R_min = 0.50 m) */

/* 7.4 Bo dieu khien truot tich phan Integral SMC (Doria-Cerezo et al. ECC 2019)
 *     Tham so ben vung (Robust) chung cho ca He A (5S) & He B (7S), moi mat san (phang & gach)
 *     va moi dai tai 0..2 kg: zeta_min = 0.48 danh dinh, >= 0.32 tren toan mien bat dinh */
#define OUTER_SMC_BETA_DEFAULT      0.016f          /* He so beta trong sigma = e2 + beta*e3 + z [m/rad] (giam hoi tiep duong c_hat) */
#define OUTER_SMC_K_DEFAULT         0.19f           /* He so chuyen mach k > 0 [m/s] */
#define OUTER_SMC_PHI_BL_DEFAULT    0.026f          /* Do day lop bien bao hoa [m] (k/phi = 7.3 -> du tru pha cao, het lac lu) */
#define OUTER_SMC_KI_DEFAULT        1.05f           /* He so tich phan ki trong z_dot = ki * e2 [1/s] (triet sai so tinh cua U) */
#define OUTER_SMC_Z_MAX_DEFAULT     0.008f          /* Gioi han chong bao hoa tich phan z [m] */
#define OUTER_SMC_W_MAX_DEFAULT     2.60f           /* Gioi han w_cmd mac dinh [rad/s] (cua R=0.5m o 0.5m/s chi can 1.0) */
#define OUTER_SMC_DW_MAX_STEP       1.25f           /* Slew-rate gioi han |delta w_cmd| moi nhip SMC [rad/s] chong dap 100% PWM */
#define OUTER_SMC_INT_DELAY_S       0.30f           /* Chi bat tich phan z sau 0.3 s khoi hanh */
/*
 * 7.5 Lap lich chu ky vong ngoai SMC thich nghi theo trang thai xac lap vong trong:
 *   - Ts_min = 0.055 s (~60 ms o nhip 100 Hz, dam bao > 0.05 s theo yeu cau thiet ke)
 *   - Ts_max = 0.090 s (< 0.20 s, chan tren tre pha de chong lac duoi fishtailing)
 *   - Khi ca 2 banh da nam trong dai |v_ref - v_act| <= max(settle_band, 8% v_ref) lien tuc du
 *     settle_margin (10 ms) VA da qua Ts_min -> phat lenh SMC moi ngay!
 *   - Ngoai ra co che early-release: neu da qua Ts_min va xe vua cat qua tam line (doi dau e2)
 *     hoac w_cmd dang nguoc chieu can sua -> cap nhat ngay, khong giu lenh cu gay vang!
 */
#define OUTER_SMC_TS_MIN_S          0.055f          /* Chu ky toi thieu SMC [s] (thuc te 60 ms > 50 ms) */
#define OUTER_SMC_TS_MAX_S          0.090f          /* Chu ky toi da SMC [s] (< 0.20 s) */
#define OUTER_SMC_SETTLE_MARGIN_S   0.010f          /* Le an toan sau khi vong trong xac lap [s] */
#define OUTER_SMC_SETTLE_BAND_MPS   0.035f          /* Dai xac lap van toc moi banh [m/s] */
/* 7.6 Quy hoach hinh hoc sa ban, giam toc & moc dung TRUC BANH CHU DONG */
#define TRACK_LEG1_DIST_M           2.000f          /* Xuat phat (3000) -> Tram tai hang (1000) [m] */
#define TRACK_LEG2_DIST_M           4.7091f         /* Tram tai hang (1000) -> Ket thuc (3000) [m] */
/*
 * Cua so Odometry chang 2 (tinh tu moc s_enc = 0 khi xe roi TRAM TAI HANG):
 *  - Do xe tien them STATION_STOP_ADVANCE_M = 100 mm ke tu tam vach tai hang (truc banh nam
 *    truoc tam vach +45 mm de toan bo than xe dai 240 mm nam gon trong vung tai hang +-150 mm),
 *    moi moc quang duong chang 2 (nga ba chu Y & vach dich) dich som hon STATION_EXTRA_ADV_M = 45 mm.
 */
#define STATION_STOP_ADVANCE_M      0.100f          /* Quang duong tien them sau tam vach tai hang [m] (100 mm) */
#define STATION_EXTRA_ADV_M         (STATION_STOP_ADVANCE_M - ROBOT_SENSOR_OFFSET_M) /* +45 mm so voi truc banh */

#define TRACK_Y_WINDOW_MIN_M        2.00f
#define TRACK_Y_WINDOW_MAX_M        2.65f
#define TRACK_Y_DIP_START_M         1.80f           /* Bat dau giam toc nhe qua nga ba chu Y */
#define TRACK_Y_DIP_END_M           2.90f
#define TRACK_Y_DIP_DELTA_V_MPS     0.18f           /* Giam toi da 0.18 m/s tai giua nga ba (0.50 -> 0.32 m/s) */
/* Quy hoach giam toc gon ve v_align = 0.16 m/s truoc tram & dich (a_dec ~ 0.83 m/s^2 <= a_max 1.0 m/s^2).
 * Cat giam tiep >50% thoi gian bo creep-speed (~0.18 s o tram, ~0.40 s o dich) ma van dung +-1 mm! */
#define TRACK_ALIGN_SPEED_MPS       0.16f           /* Van toc tiep can & can chinh truc banh [m/s] (d_brake ~4..5 mm) */
#define TRACK_DECEL_WINDOW_M        0.150f          /* Bat dau giam toc truoc dich 150 mm (cu: 180 mm) */
#define TRACK_DECEL_MARGIN_M        0.015f          /* Dem bo o v_align truoc khi cam bien cham vach (cu: 30 mm) */
#define TRACK_LEG2_1KG_ODOM_COMP_M  0.015f          /* Bu nen lop odometry chang 2 khi cho 1 kg (+15 mm) */
#define TRACK_LEG2_2KG_ODOM_COMP_M  0.060f          /* Bu nen lop odometry chang 2 khi cho 2 kg (+60 mm) */
/*
 * Phat hien vach cat ngang (Tram tai hang: rong 26 mm x dai 50 mm):
 *  - Profile 7 mat: >= 5 mat sang (I_eff > 0.15) & total_I > 3.20
 *  - Profile 5 mat: >= 4 mat sang (I_eff > 0.15) & total_I > 2.65
 *  - Sau khi cam bien vuot qua vach ngang, tinh tam vach s_cb_center = 0.5*(s_in + s_out),
 *    xe tien them (d1 - brake_lead) de TRUNG DIEM TRUC BANH CHU DONG dung dung X = 1000 +- 5 mm.
 */
#define TRACK_MARKER_WIN_MIN_M      1.65f           /* Chi mo cong nhan vach ngang tram tai tu s >= 1.65 m (tranh nhan nham) */
#define MARKER_CONFIRM_TICKS_200HZ  3               /* 3 * 5 ms = 15 ms chong nhieu */
#define MARKER_MIN_ACTIVE_7S        5
#define MARKER_TOTAL_I_THR_7S       3.60f           /* Cao hon 1 vach don (~2.85) de khong nhan som truoc vach ngang */
#define MARKER_MIN_ACTIVE_5S        4
#define MARKER_TOTAL_I_THR_5S       3.30f           /* Vach ngang 50 mm thuc do I ~ 3.8..4.4 -> chi bat dung vach ngang */
#define MARKER_AXLE_TRIM_M          0.000f          /* Bu vi chinh thuc nghiem tram tai [m] (+ tien them) */
/*
 * Phat hien dau mut ket thuc cuoi duong (khong vach ngang, X = 3000 mm):
 *  - Khi truc banh di duoc ~4.654 m, day cam bien (truoc 55 mm) cham dau mut X = 3000 mm.
 *  - Mo cong EOL tu s_enc >= 4.45 m, fail-safe o s_enc >= 4.78 m.
 *  - Khi total_I < EOL_TOT_THRESH (hoac mat line) du 2 nhip 200 Hz (10 ms), khoa lai thang
 *    (w_cmd = 0) va bo tiep (d1 - EOL_PSF_OFFSET - brake_lead) de TRUC BANH dung tai X = 3000 +- 5 mm.
 */

#define TRACK_GOAL_WIN_MIN_M        4.50f
#define TRACK_GOAL_WIN_MAX_M        4.78f
#define GOAL_EOL_TOT_THRESH_7S      1.65f           /* Nguong sut tong sang khi cam bien cham mep cuoi (7S) */
#define GOAL_EOL_TOT_THRESH_5S      1.50f           /* Nguong sut tong sang (5S) */
#define GOAL_EOL_CONFIRM_TICKS      3               /* 3 * 5 ms = 15 ms xac nhan het vach (tranh bat som) */
#define GOAL_EOL_PSF_OFFSET_M       0.003f          /* Bu tre quang hoc PSF + 10 ms debounce (~3 mm) */
#define GOAL_AXLE_TRIM_M            0.000f          /* Bu vi chinh thuc nghiem vach dich [m] (+ tien them) */
/* 7.7 Nhan dien tai trong tai Tram tai hang (HX711) */
#define LOAD_DETECT_MIN_G           700.0f          /* Nguong nhan biet da dat hang (> 700 g) */
#define LOAD_STABLE_BAND_G          50.0f           /* Bien do dao dong on dinh (+-50 g) */
#define LOAD_STABLE_HOLD_S          1.20f           /* Thoi gian on dinh lien tuc (1.0 - 1.5 s) */
#define LOAD_CLASS_SPLIT_G          1500.0f         /* < 1500 g -> 1 kg (Trai), >= 1500 g -> 2 kg (Phai) */
/* 7.8 Nguong dung an toan & ket thuc nhiem vu [Stop] */
#define SAFETY_LINE_LOST_STOP_S     2.0f            /* Mat line lien tuc >= 2.0 s -> dung xe */
#define SAFETY_ERR_LIMIT_M          0.016f          /* Sai so vuot > 16 mm... */
#define SAFETY_ERR_LIMIT_STOP_S     2.0f            /* ...lien tuc >= 2.0 s -> dung xe */
#define GOAL_STOP_HOLD_S            2.0f            /* Dung tai vach dich du 2.0 s -> hoan tat / burst */