#pragma once

/*
 * ============================================================================
 * robot_params.h - Tap hop tap trung thong so Dong hoc & Dong luc hoc Robot
 * ============================================================================
 *
 * NGUYEN TAC QUY CHUAN (Single Source of Truth):
 *  - Moi thong so hinh hoc, co-dien, mo hinh ham truyen nhan dang va luat
 *    tham so bo dieu khien vong trong 2-DOF PI duoc dat duy nhat tai day.
 *  - Khi co xung dot giua thiet lap thuc nghiem `Motor_PI_Tuning` (da hieu
 *    chuan tren phan cung that & mat bat sa ban) va file mo phong MATLAB
 *    `SMC_controller14.m`, TA LAY `Motor_PI_Tuning` LAM CHUAN:
 *      1) Khoang cach 2 banh xe B = 0.190 m (thuc te PCB/khung V2) thay vi 0.180 m trong SMC.
 *      2) Tan so lay mau vong trong fs = 200 Hz (Ts = 5 ms) thay vi 100 Hz trong SMC.
 *      3) Mo hinh dong co va tham so 2-DOF PI dung don vi [m/s] & luat thich nghi
 *         theo tai trong m_load tren mat bat sa ban (w_gc = 30 rad/s), thay vi
 *         mo hinh cu theo RPM trong SMC.
 *      4) Quy uoc kenh dong co: Dong co A = banh TRAI (Left), Dong co B = banh PHAI (Right).
 *         (Ca 2 kenh cung mang dau duong khi xe tien ve phia truoc.)
 * ============================================================================
 */

/* -------------------------------------------------------------------------- */
/* 1. THONG SO HINH HOC & DONG HOC KHUNG XE (Kinematics & Geometry)           */
/* -------------------------------------------------------------------------- */

/* Banh xe chu dong V2: duong kinh D = 65 mm, ban kinh R = 32.5 mm */
#define ROBOT_WHEEL_DIAMETER_M      0.065f
#define ROBOT_WHEEL_RADIUS_M        0.0325f
#define ROBOT_WHEEL_CIRCUM_M        0.20420352f     /* C = pi * D [m] */

/* Quy doi toc do quay truc banh xe [RPM] <-> van toc dai [m/s] */
#define ROBOT_RPM_TO_MPS            (ROBOT_WHEEL_CIRCUM_M / 60.0f)
#define ROBOT_MPS_TO_RPM            (s60.0f / ROBOT_WHEEL_CIRCUM_M)

/*
 * Khoang cach tam hai banh chu dong (Track width B) [m].
 * CHUAN Motor_PI_Tuning / Khung V2: B = 190 mm = 0.190 m
 * (Luu y: file mo phong SMC_controller14.m dung L = 0.180 m -> da cap nhat ve 0.190 m).
 *
 * Quan he dong hoc vi sai (he truc tay phai, omega > 0 la re trai / CCW):
 *   Dong hoc nghich (tinh tu lenh vong ngoai v_cmd, w_cmd):
 *     v_left_ref  (Motor A) = v_cmd - (B / 2) * w_cmd
 *     v_right_ref (Motor B) = v_cmd + (B / 2) * w_cmd
 *   Dong hoc thuan (tinh odometry tra ve cho Observer/SMC):
 *     v_act = 0.5 * (v_left + v_right)
 *     w_act = (v_right - v_left) / B
 */
#define ROBOT_TRACK_WIDTH_M         0.180f
#define ROBOT_HALF_TRACK_M          (ROBOT_TRACK_WIDTH_M * 0.5f)

/* Kich thuoc tong the khung xe & day cam bien do line (phuc vu vong ngoai SMC) */
#define ROBOT_BODY_LENGTH_M         0.200f          /* Chieu dai than xe [m] */
#define ROBOT_BODY_WIDTH_M          0.136f          /* Chieu rong than xe [m] */
#define ROBOT_SENSOR_OFFSET_M       0.055f          /* d1: khoang cach truc banh -> day cam bien [m] */
#define ROBOT_LINE_WIDTH_M          0.026f          /* Be rong vach line den [m] */
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
 * Vong trong (Inner Velocity Loop - 2-DOF PI + M/T Encoder):
 * CHUAN Motor_PI_Tuning: fs = 200 Hz, Ts = 5 ms (tre ZOH ~2.5 ms, fs/f_BW > 35).
 * Chay doc quyen tren Core 1 bang ngat cung GPTimer.
 */
#define INNER_LOOP_FS_HZ            200U
#define INNER_LOOP_TS_US            5000U
#define INNER_LOOP_TS_S             0.005f

/*
 * Tan so tham chieu cho cac tang vong ngoai (theo SMC_controller14.m),
 * co the dong bo bang bo chia nhieu tu nhip 200 Hz cua vong trong:
 *   - Bo quan sat Luenberger: 100 Hz (moi 2 chu ky 5 ms)
 *   - Bo dieu khien truot SMC: 20 Hz (moi 10 chu ky 5 ms)
 */
#define OBSERVER_LOOP_FS_HZ         100U
#define OBSERVER_LOOP_TS_S          0.010f
#define OBSERVER_LOOP_DIVIDER       (INNER_LOOP_FS_HZ / OBSERVER_LOOP_FS_HZ)  /* 2 */

#define OUTER_SMC_FS_HZ             20U
#define OUTER_SMC_TS_S              0.050f
#define OUTER_SMC_DIVIDER           (INNER_LOOP_FS_HZ / OUTER_SMC_FS_HZ)      /* 10 */

/* -------------------------------------------------------------------------- */
/* 4. THONG SO DONG LUC HOC DONG CO DA NHAN DANG (Identified Plant Models)    */
/* -------------------------------------------------------------------------- */
/*
 * Mo hinh bac 1 tach vung chet:
 *   u_eff = max(0, u - u_dz)
 *   G1(s) = K_dc / (tau_m * s + 1)   [ngo vao: %PWM, ngo ra: m/s]
 *
 * A) Tren ga do (khong tai, banh quay tu do - Bang 5.3):
 *    - Motor A (trai): K_dc = 0.01175 (m/s)/%, tau_m = 0.0762 s, u_dz = 2.40 %
 *    - Motor B (phai): K_dc = 0.01225 (m/s)/%, tau_m = 0.0952 s, u_dz = 1.68 %
 *
 * B) Tren mat bat sa ban (co tai trong m_load in [0..2] kg - Muc 6.1.3):
 *    - Motor A (trai): tau_A(m) = 0.1128 + 0.0262*m [s], u_dz_A(m) = 4.51 + 2.78*m [%]
 *    - Motor B (phai): tau_B(m) = 0.1328 + 0.0324*m [s], u_dz_B(m) = 7.32 + 5.21*m [%]
 */
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
/*
 * Phuong phap Bode khu cuc co hoc (z_I = 1/tau_m) voi tre an Td ~ 8-10 ms:
 *   - Tren mat bat sa ban (MAC DINH khi chay that): w_gc = 30 rad/s (f_BW = 4.77 Hz),
 *     PM ~ 72.8..76.2 deg, Ki = 0.80 * w_gc * K_ff, tranh cong huong lop ~28 Hz o tai 2 kg.
 *     Cong thuc cap nhat theo khoi luong hang m_load [kg] tu HX711:
 *       Kp_A(m)    = 291.0 + 67.6 * m   [%PWM / (m/s)]
 *       u_dz_A(m)  =  4.51 + 2.78 * m   [%PWM]
 *       Kp_B(m)    = 326.7 + 79.7 * m   [%PWM / (m/s)]
 *       u_dz_B(m)  =  7.32 + 5.21 * m   [%PWM]
 *       e_band(m)  = 0.020 + 0.005 * m  [m/s]
 */
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
#define ROBOT_V_NOM_MPS             0.50f           /* Van toc dai danh dinh [m/s] */
#define ROBOT_V_WHEEL_MIN_OP_MPS    0.30f           /* Can duoi vung cua (R_min = 0.5 m) [m/s] */
#define ROBOT_V_WHEEL_MAX_OP_MPS    0.70f           /* Can tren vung cua (R_min = 0.5 m) [m/s] */
#define ROBOT_V_WHEEL_ABS_MAX_MPS   0.85f           /* Gioi han bao ve van toc dat moi banh [m/s] */
#define ROBOT_W_MAX_RADS            4.50f           /* Gioi han van toc goc dat cua vong ngoai SMC [rad/s] */
#define ROBOT_A_MAX_MPS2            1.00f           /* Gia toc cuc dai chong truot banh [m/s^2] */
#define ROBOT_MIN_TURN_RADIUS_M     0.50f           /* Ban kinh cua nho nhat tren sa ban [m] */

/*
 * Quy dao khoi dong muot S-curve bac 3 (Muc 6.1.1):
 *   v_ref(tau) = v0 + (v1 - v0) * (3*tau^2 - 2*tau^3), tau = t / T_ramp in [0, 1]
 *   Gia toc cuc dai tai tau = 0.5: a_max = 1.5 * |v1 - v0| / T_ramp.
 *   Voi 0 -> 0.5 m/s trong 0.80 s: a_max = 0.9375 m/s^2 <= 1.0 m/s^2.
 */
#define ROBOT_SCURVE_DEFAULT_T_S    0.80f

/* Nguong nhan dien xe da dung han (su dung cho may trang thai tram tai hang / ve dich) */
#define ROBOT_STOP_RPM_THR          0.50f           /* [RPM] ~ 1.7 mm/s */

/* Quang duong phanh Short Brake thuc do tu 0.7 -> 0 m/s tren bat (Bang 6.6 - tham khao) */
#define ROBOT_STOP_DIST_0KG_M       0.064f          /* TB A/B: 57.9 / 69.8 mm */
#define ROBOT_STOP_DIST_1KG_M       0.071f          /* TB A/B: 65.0 / 76.9 mm */
#define ROBOT_STOP_DIST_2KG_M       0.078f          /* TB A/B: 72.0 / 84.0 mm */
