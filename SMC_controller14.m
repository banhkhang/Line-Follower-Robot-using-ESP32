%% MO PHONG ROBOT BAM LINE SMC - 1 DAY CAM BIEN CO NHIEU (1000 Hz), BO QUAN SAT LUENBERGER & TACH NHANH GAUSS-NEWTON (100 Hz) & SMC (20 Hz)
%% MAT TRUOT THEO BAI BAO DORIA-CEREZO ET AL. (ECC 2019), SUA DAU & KHOI TAO QUAN SAT CHO DUNG QUY UOC PHUONG TIEN
%% CAP NHAT: CHUYEN THUAT TOAN TACH NHANH GAUSS-NEWTON SANG CHAY O 100 Hz DE GIAM 90% TAI CPU TREN ESP32
clear; clc; close all;
rng(42); % Co dinh hat giong ngau nhien de de doi chieu ket qua

%% 1. THONG SO CO KHI, HINH HOC XE & 1 DAY CAM BIEN DUY NHAT (d1 = 0.055 m)
r = 0.0325;             % Ban kinh banh xe (m) = 32.5 mm -> duong kinh 65 mm
L = 0.180;              % Khoang cach 2 banh xe (m) = 180 mm
d1 = 0.055;             % Khoang cach tu truc banh xe den day cam bien duy nhat (m) = 55 mm
v_req = 0.4;            % Van toc dai yeu cau (m/s) (>= 0.1 m/s theo de bai)
line_width = 0.026;     % Be rong duong line (m) = 26 mm

% Kich thuoc ve hinh tuong trung robot (don vi: mm)
body_len   = 200;       % Chieu dai than xe = 200 mm
body_wid   = 136;       % Chieu rong than xe = 136 mm
wheel_diam = 2 * r * 1000; % Duong kinh banh xe = 65 mm
wheel_wid  = 22;        % Be rong banh xe = 22 mm

% Cau hinh 1 day cam bien hong ngoai 7 mat (pitch = 9 mm)
y_sens    = (-3:3) * 0.009; % [-27, -18, -9, 0, 9, 18, 27] mm
noise_std = 0.022;          % Do lech chuan nhieu ADC 10-bit + anh sang moi truong
ch_gain   = [0.96, 1.03, 0.98, 1.02, 0.97, 1.04, 0.95]; % Sai lech do nhay giua 7 mat

% Tham so cong encoder & quang hoc cho vung tach nhanh chu Y (tach doi tai X = 668.63 mm)
S_Y_WINDOW_MIN  = 2.15; % (m) Mo cong nhan dien nga ba chu Y sau khi roi tram tai hang
S_Y_WINDOW_MAX  = 2.70; % (m) Dong cong nhan dien nga ba chu Y
SINGLE_LINE_TOT = 2.36; % Nguong tong cuong do sang toi da cua 1 vach don 26 mm

% Tham so phat hien ket thuc duong line (khong co vach ngang cuoi duong)
EOL_TOT_THRESH    = 1.98; % Nguong tong cuong do sang khi day cam bien cham mep cuoi line
EOL_CONFIRM_TICKS = 1;    % Xac nhan mat vach lien tiep 1 ms (o 1000 Hz)

%% 2. THONG SO DA TAN SO: CAM BIEN (1000 Hz) - VONG TRONG OBSERVER & GAUSS-NEWTON (100 Hz) - SMC (20 Hz)
Ts_sens = 0.001;        % Chu ky lay mau cam bien & tich phan vat ly (s) = 1000 Hz
Ts_in   = 0.01;         % Chu ky lay mau vong trong 2-DOF PI + Observer + Gauss-Newton (s) = 100 Hz
Ts_out  = 0.05;         % Chu ky lay mau vong ngoai SMC (s) = 20 Hz

N_in    = round(Ts_in / Ts_sens);   % 10 buoc cam bien / 1 chu ky vong trong (100 Hz)
N_out   = round(Ts_out / Ts_sens);  % 50 buoc cam bien / 1 chu ky vong ngoai (20 Hz)

KL = 38.41; aL = 11.27; % Dong co trai: 38.41/(s+11.27)
KR = 30.61; aR = 9.436; % Dong co phai: 30.61/(s+9.436)
dz = 10.0;              % Vung chet +-10% duty cycle
u_max = 100.0;          % Bao hoa +-100% duty cycle

% Roi rac hoa doi tuong dong co o buoc vat ly Ts_sens = 0.001 s
adL = exp(-aL * Ts_sens); bdL = (KL / aL) * (1 - adL);
adR = exp(-aR * Ts_sens); bdR = (KR / aR) * (1 - adR);

% Tham so 2-DOF PI + Feedforward + Anti-windup cho vong trong (Ts_in = 0.01 s)
zeta = 0.92; wn = 46.0; b_2dof = 0.1;
KpL = (2*zeta*wn - aL)/KL; KiL = (wn^2)/KL; KffL = aL/KL;
KpR = (2*zeta*wn - aR)/KR; KiR = (wn^2)/KR; KffR = aR/KR;
Kb  = 1.0 / Ts_in;

%% 3. BO LOC ALPHA-BETA (1000 Hz), BO QUAN SAT LUENBERGER THICH NGHI (100 Hz) & SMC (20 Hz)
wn_f     = 75.0;        % Tan so tu nhien bo loc (rad/s) giam nhieu tan so cao
zeta_f   = 0.95;        % He so tat dan toi uu
alpha_ab = 1.0 - exp(-2.0 * zeta_f * wn_f * Ts_sens);
beta_ab  = (wn_f * Ts_sens)^2;

% He so bo loc Alpha-Beta o 100 Hz khi dang trong vung tach nhanh chu Y
alpha_ab_in = 1.0 - exp(-2.0 * zeta_f * wn_f * Ts_in);
beta_ab_in  = (wn_f * Ts_in)^2;

% Tham so Bo quan sat Luenberger thich nghi (100 Hz) & Integral SMC (20 Hz)
tau_c       = 0.035;    % Hang so thoi gian loc uoc luong do cong c_hat (s)
t_ramp      = 0.25;     % Thoi gian khoi hanh mem (s)
v_min_denom = 0.30;     % Gioi han mau so van toc on dinh do loi quan sat

beta_smc = 0.016;       % He so beta trong mat truot sigma = d + beta*e3 + z
k_smc    = 0.30;        % He so chuyen mach k > 0 trong luat dieu khien truot
phi_bl   = 0.015;       % Lop bien bao hoa chong rung thay cho sign(sigma) (m)
ki_smc   = 1.70;        % He so tich phan ki trong z_dot = ki * d
z_max    = 0.015;       % Gioi han chong bao hoa tich phan z (m)
w_max    = 4.5;         % Gioi han van toc goc dat (rad/s)

% Cuc bo quan sat Luenberger thich nghi cho Ts_in = 0.01 s (100 Hz)
p1_slow = 0.89; p2_slow = 0.92; % Loc nhieu cam bien khi bam thang/cua deu o 100 Hz
p1_fast = 0.75; p2_fast = 0.80; % Bat nhanh goc e3 khi chuyen tiep cua o 100 Hz

% Thong so quy hoach van toc & giam toc dung chinh xac (+-5 mm)
v_min      = 0.10;      % Van toc di chuyen toi thieu (m/s)
D_dec      = 0.22;      % Bat dau giam toc truoc dich 220 mm
d_margin   = 0.018;     % Khoang dem bo o toc do 0.1 m/s truoc diem ket thuc (18 mm)
a_dec      = (v_req^2 - v_min^2) / (2 * (D_dec - d_margin));

% Quang duong chuan theo hinh hoc sa ban de bai (2 nhanh doi xung nen chieu dai bang nhau)
S_leg1     = 2.000;     % Quang duong tu Xuat phat (3000) -> Tram tai hang (1000) (m)
R_s        = 0.800;
alpha_s    = pi / 4;    % Goc chuyen huong 45 deg
T_s        = R_s * tan(alpha_s / 2); % Khoang lui tiep tuyen ~0.33137 m
L_diag_mid = 0.500 * sqrt(2) - 2 * T_s; % Doan thang xien 45 deg giua 2 cung R800 (~0.04437 m)
S_leg2     = 0.500 + 0.500*pi + (0.500 - T_s) + 2*(R_s * alpha_s) + L_diag_mid + (1.500 - T_s); % ~4.7091 m

% Cua so encoder cho phep kich hoat phat hien het vach o cuoi chang 2
S_GOAL_WIN_MIN = S_leg2 - 0.15; % (m) Mo cong tim dau mut ket thuc (~4.559 m)
S_GOAL_WIN_MAX = S_leg2 + 0.08; % (m) Gioi han dung du phong fail-safe (~4.789 m)
s_enc          = 0;

%% 4. XAY DUNG BAN DO SA BAN CHUAN THEO DE BAI
[common_pts, branch1_pts, branch2_pts, marker_x] = build_track_map();
all_map_pts = [common_pts; branch1_pts; branch2_pts]; % Toan bo vach den cho cam bien quet
target_pts  = [common_pts; branch1_pts];              % Quy dao chuan de doi chieu sai so thuc

%% 5. KHOI TAO TRANG THAI & CUA SO ANIMATION (FIGURE 1)
e2_init = 0.005;        % Lech ngang ban dau +5 mm
e3_init = deg2rad(15);  % Lech goc ban dau +15 deg
phi = pi - e3_init;
x = 3.000 - d1 * cos(phi);
y = -e2_init - d1 * sin(phi);

rpmL = 0; rpmR = 0;
intL = 0; intR = 0;
uL_eff = 0; uR_eff = 0;
w_cmd = 0; v_cmd = 0;
v_act = 0; w_act = 0;

% Khoi tao trang thai bo loc cam bien Alpha-Beta & bo quan sat Luenberger
[e2_true_ref, e3_act] = calc_exact_errors(x + d1*cos(phi), y + d1*sin(phi), phi, target_pts);
e1_filt     = e2_true_ref;
de1_filt    = 0.0;

e2_hat      = e2_true_ref;
e3_hat      = e3_act;
c_hat       = 0.0;
z_int       = 0.0;
t_start_seg = 0.0;

e2_meas_ctrl = e1_filt;
e3_est       = e3_hat;

state = 1;              % 1: Chay den tram | 2: Cho nhan hang | 3: Chay ve dich | 4: Dung
station_done = false;
marker_hits  = 0;       % Bo dem chong nhieu 1 mau tai mep dau vach dung
marker_seen  = false;
eol_hits     = 0;       % Bo dem xac nhan het vach tai dich
eol_reached  = false;   % Co bao da cham dau mut ket thuc cuoi duong
load_kg = 1;
wait_timer = 0;

fig1 = figure('Name', 'Figure 1: Animation Quy dao Robot Bam Line (Gauss-Newton 100 Hz)', 'Position', [60, 120, 820, 620]);
ax1 = axes(fig1); hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
plot(ax1, common_pts(:,1)*1000, common_pts(:,2)*1000, 'k-', 'LineWidth', 3);
plot(ax1, branch1_pts(:,1)*1000, branch1_pts(:,2)*1000, 'k-', 'LineWidth', 3);
plot(ax1, branch2_pts(:,1)*1000, branch2_pts(:,2)*1000, 'k-', 'LineWidth', 3);
plot(ax1, [1000 1000], [-25 25], 'k-', 'LineWidth', 4);
rectangle(ax1, 'Position', [850, -150, 300, 300], 'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1.2);

plot(ax1, [500 1000 1500], [1000 1000 1500], 'k:', 'LineWidth', 0.8);
plot(ax1, [500 1000 1500], [1000 1000 500],  'k:', 'LineWidth', 0.8);

plot(ax1, 3000, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(ax1, 3000, 1500, 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(ax1, 3000, 500, 'ms', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
text(ax1, 2720, -90, 'Bat dau', 'FontWeight', 'bold');
text(ax1, 800, -220, 'Khu vuc tai hang (2000 mm)', 'Color', 'r', 'FontWeight', 'bold');
text(ax1, 2550, 1580, 'Ket thuc (1 kg)', 'Color', 'r', 'FontWeight', 'bold');
text(ax1, 2550, 410, 'Ket thuc (2 kg)', 'Color', 'm', 'FontWeight', 'bold');
xlim(ax1, [-150 3250]); ylim(ax1, [-300 1750]);
xlabel(ax1, 'X (mm)'); ylabel(ax1, 'Y (mm)');

h_traj   = plot(ax1, NaN, NaN, 'b--', 'LineWidth', 1.5);
h_boom   = plot(ax1, NaN, NaN, 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
h_sens1  = plot(ax1, NaN, NaN, 'g-', 'LineWidth', 3);
h_body   = patch(ax1, NaN, NaN, [0.15 0.55 0.9], 'EdgeColor', 'k', 'LineWidth', 1.2);
h_wheelL = patch(ax1, NaN, NaN, [0.15 0.15 0.15], 'EdgeColor', 'k');
h_wheelR = patch(ax1, NaN, NaN, [0.15 0.15 0.15], 'EdgeColor', 'k');
h_title  = title(ax1, 'Mo phong Robot Bam Line (Gauss-Newton 100Hz + Observer 100Hz)');

update_robot_graphics(x, y, phi, body_len, body_wid, L*1000, wheel_diam, wheel_wid, ...
    d1*1000, h_body, h_wheelL, h_wheelR, h_boom, h_sens1);
drawnow;

%% 6. VONG LAP MO PHONG DA TAN SO (1000 Hz / 100 Hz / 20 Hz)
t_max = 35.0;
N_steps = round(t_max / Ts_sens);

N_log_max     = ceil(N_steps / N_in);
log_t         = zeros(N_log_max, 1);
log_x         = zeros(N_log_max, 1);
log_y         = zeros(N_log_max, 1);
log_phi       = zeros(N_log_max, 1);
log_e2_true   = nan(N_log_max, 1);
log_e2_filt   = nan(N_log_max, 1);
log_e2_raw    = nan(N_log_max, 1);
log_e3_est    = nan(N_log_max, 1);
log_e3_act    = nan(N_log_max, 1);
log_sigma     = nan(N_log_max, 1);
log_sat_sigma = nan(N_log_max, 1);
log_rpmL      = zeros(N_log_max, 1);
log_rpmR      = zeros(N_log_max, 1);
log_rpmL_ref  = zeros(N_log_max, 1);
log_rpmR_ref  = zeros(N_log_max, 1);
log_y_active  = zeros(N_log_max, 1);

log_idx = 0;
anim_skip_log = 4; % Cap nhat animation moi 0.04 s
gn_calls_count = 0; % Dem so lan goi Gauss-Newton de so sanh tai CPU

for step = 1:N_steps
    t = (step - 1) * Ts_sens;
    
    % =====================================================================
    % TANG 1: DOC CAM BIEN 7 MAT (1000 Hz) & LOC ALPHA-BETA DUONG DON
    % =====================================================================
    x1 = x + d1 * cos(phi);
    y1 = y + d1 * sin(phi);
    
    e1_pred = e1_filt + de1_filt * Ts_sens;
    
    % Tang 1000 Hz chi doc mang cam bien I_eff va tinh trong tam duong don (KHONG chay Gauss-Newton)
    [e_lat1_raw, is_marker1, total_I, I_eff] = read_sensor_1000hz(x1, y1, phi, all_map_pts, ...
        y_sens, line_width, marker_x, ~station_done, noise_std, ch_gain);
    
    if isnan(e_lat1_raw)
        e_lat1_raw = e1_pred;
    end
    if is_marker1
        marker_hits = marker_hits + 1;
        if marker_hits >= 5
            marker_seen = true;
        end
    end
    
    % Phat hien dau mut ket thuc cuoi duong (khong vach ngang) bang Odometry Gating + Mat vach
    if state == 3 && s_enc >= S_GOAL_WIN_MIN
        if total_I < EOL_TOT_THRESH
            eol_hits = eol_hits + 1;
            if eol_hits >= EOL_CONFIRM_TICKS
                eol_reached = true;
            end
        else
            eol_hits = 0;
        end
        
        if s_enc >= S_GOAL_WIN_MAX
            eol_reached = true;
        end
    else
        eol_hits    = 0;
        eol_reached = false;
    end
    
    % Kiem tra xem xe co dang o trong cua so ngã ba chu Y hay khong
    in_y_zone = (state == 3) && (s_enc >= S_Y_WINDOW_MIN) && (s_enc <= S_Y_WINDOW_MAX);
    
    if ~in_y_zone
        % Duong don binh thuong: cap nhat bo loc Alpha-Beta o 1000 Hz
        res1     = e_lat1_raw - e1_pred;
        e1_filt  = e1_pred + alpha_ab * res1;
        de1_filt = de1_filt + (beta_ab / Ts_sens) * res1;
    else
        % Trong vung nga ba chu Y: chi du doan tuyen tinh o 1000 Hz, cho Gauss-Newton cap nhat o 100 Hz
        e1_filt = e1_pred;
    end
    
    % =====================================================================
    % TANG 2 & 3: VONG TRONG 100 Hz (GAUSS-NEWTON TACH NHANH + OBSERVER + 2-DOF PI) & SMC 20 Hz
    % =====================================================================
    if mod(step - 1, N_in) == 0
        is_y_active = false;
        
        % Chay thuật toán tách nhánh Gauss-Newton ở 100 Hz khi nằm trong vùng ngã ba chữ Y
        if in_y_zone && total_I >= 0.25
            e_pred_100hz = e1_filt; % Da tich phan du doan qua 10 buoc 1 ms (= 10 ms)
            [e_y_100hz, is_y_active] = solve_y_branch_100hz(I_eff, y_sens, total_I, ...
                e_pred_100hz, load_kg, line_width, SINGLE_LINE_TOT);
            
            if is_y_active
                gn_calls_count = gn_calls_count + 1;
            end
            
            % Cap nhat bo loc Alpha-Beta theo nhip 100 Hz de giu van toc ngang de1_filt chinh xac
            res_in   = e_y_100hz - e_pred_100hz;
            e1_filt  = e_pred_100hz + alpha_ab_in * res_in;
            de1_filt = de1_filt + (beta_ab_in / Ts_in) * res_in;
            e_lat1_raw = e_y_100hz; % Ghi log gia tri da tach nhanh
        end
        
        e2_meas_ctrl = e1_filt;
        
        % Tinh sai so hinh hoc chuan chi de ve do thi & danh gia ket qua
        [e2_tmp, e3_tmp] = calc_exact_errors(x1, y1, phi, target_pts);
        if ~isempty(e2_tmp), e2_true_ref = e2_tmp(1); end
        if ~isempty(e3_tmp), e3_act = e3_tmp(1); end
        
        if state == 1
            d_rem  = S_leg1 - s_enc;
            v_prof = calc_decel_velocity(d_rem, v_req, v_min, D_dec, d_margin, a_dec);
            v_ramp = v_req * min(1.0, (t - t_start_seg) / t_ramp);
            v_cmd  = min(v_prof, v_ramp);
            
            % Dung chinh xac tai tam vach ngang 30x50 mm (suon ra cua marker)
            if ~station_done && ((marker_seen && ~is_marker1) || (x1 <= marker_x && abs(y1) < 0.1))
                state = 2;
                station_done = true;
                v_cmd = 0; w_cmd = 0;
                fprintf('Sai so dung tai Tram tai hang: %+.2f mm\n', (marker_x - x1)*1000);
            end
        elseif state == 2
            v_cmd = 0; w_cmd = 0;
            wait_timer = wait_timer + Ts_in;
            if wait_timer >= 0.3 && abs(rpmL) < 0.5 && abs(rpmR) < 0.5
                drawnow;
                choice = questdlg('Robot da dung tai Khu vuc tai hang (2000 mm). Chon khoi luong hang dat len:', ...
                    'Phan loai hang hoa', '1 kg (Re nhanh tren)', '2 kg (Re nhanh duoi)', '1 kg (Re nhanh tren)');
                if contains(choice, '2 kg')
                    load_kg = 2;
                    target_pts = [common_pts; branch2_pts];
                else
                    load_kg = 1;
                    target_pts = [common_pts; branch1_pts];
                end
                s_enc       = 0; % Reset encoder cho chang 2
                eol_hits    = 0;
                eol_reached = false;
                state       = 3;
                t_start_seg = t;
                z_int       = 0.0;
                c_hat       = 0.0;
                e2_hat      = e1_filt;
                [~, e3_tmp] = calc_exact_errors(x + d1*cos(phi), y + d1*sin(phi), phi, target_pts);
                if ~isempty(e3_tmp), e3_hat = e3_tmp(1); end
            end
        elseif state == 3
            d_rem  = S_leg2 - s_enc;
            v_prof = calc_decel_velocity(d_rem, v_req, v_min, D_dec, d_margin, a_dec);
            v_ramp = v_req * min(1.0, (t - t_start_seg) / t_ramp);
            v_cmd  = min(v_prof, v_ramp);
            
            % Giam toc nhe & muot qua vung tach nhanh chu Y (s_enc = 1.90 -> 3.00 m)
            if s_enc > 1.90 && s_enc < 3.00
                u_junc = (s_enc - 1.90) / (3.00 - 1.90);
                v_junc = v_req - 0.16 * 0.5 * (1 - cos(2 * pi * u_junc));
                v_cmd  = min(v_cmd, v_junc);
            end
            
            % Dieu kien dung cuoi duong khong vach ngang
            if eol_reached
                state = 4;
                v_cmd = 0; w_cmd = 0;
                intL = 0; intR = 0;
            end
        elseif state == 4
            v_cmd = 0; w_cmd = 0;
        end
        
        % --- BO QUAN SAT LUENBERGER THICH NGHI TRONG VONG TRONG (Ts_in = 0.01 s = 100 Hz) ---
        if abs(v_act) > 0.04
            v_obs = v_act;
        else
            v_obs = v_cmd;
        end
        
        if abs(v_obs) > 0.04 && (state == 1 || state == 3)
            v_denom = max(abs(v_obs), v_min_denom);
            
            c_raw   = w_act / v_denom;
            alpha_c = Ts_in / (tau_c + Ts_in);
            c_hat   = (1.0 - alpha_c) * c_hat + alpha_c * c_raw;
            
            err_obs   = e2_meas_ctrl - e2_hat;
            blend_obs = min(1.0, max(0.0, (abs(err_obs) - 0.00045) / 0.0009));
            p1_obs    = (1.0 - blend_obs) * p1_slow + blend_obs * p1_fast;
            p2_obs    = (1.0 - blend_obs) * p2_slow + blend_obs * p2_fast;
            
            Ld1 = 2.0 - (p1_obs + p2_obs);
            Ld2 = (p1_obs * p2_obs - 1.0 + Ld1) / (v_denom * Ts_in);
            
            w_net       = w_act - c_hat * v_obs;
            e2_pred_obs = e2_hat + v_obs * Ts_in * sin(e3_hat) - d1 * Ts_in * w_act - 0.5 * v_obs * (Ts_in^2) * w_net;
            e3_pred_obs = e3_hat + Ts_in * (c_hat * v_obs - w_act);
            
            e2_hat = e2_pred_obs + Ld1 * err_obs;
            e3_hat = e3_pred_obs + Ld2 * err_obs;
        end
        e3_est = e3_hat;
        
        % --- VONG NGOAI SMC (Ts_out = 0.05 s = 20 Hz) ---
        if mod(step - 1, N_out) == 0
            if state == 1 || state == 3
                if (t - t_start_seg) > 0.30
                    z_int = max(-z_max, min(z_max, z_int + ki_smc * e2_meas_ctrl * Ts_out));
                end
                
                sigma     = e2_meas_ctrl + beta_smc * e3_hat + z_int;
                sat_sigma = max(-1.0, min(1.0, sigma / phi_bl));
                
                denom_smc = d1 + beta_smc;
                u_eq  = (v_cmd * (sin(e3_hat) + beta_smc * c_hat) + ki_smc * e2_meas_ctrl) / denom_smc;
                w_raw = u_eq + (k_smc / denom_smc) * sat_sigma;
                
                w_cmd = max(-w_max, min(w_max, w_raw));
            else
                w_cmd = 0.0;
                sigma = 0.0;
                sat_sigma = 0.0;
            end
        end
        
        wR_ref   = (v_cmd + w_cmd * L / 2.0) / r;
        wL_ref   = (v_cmd - w_cmd * L / 2.0) / r;
        rpmR_ref = wR_ref * 60 / (2 * pi);
        rpmL_ref = wL_ref * 60 / (2 * pi);
        
        [uL_sat, intL] = step_2dof_pi(rpmL_ref, rpmL, intL, KpL, KiL, KffL, b_2dof, Kb, dz, u_max, Ts_in);
        uL_eff = apply_deadzone(uL_sat, dz);
        
        [uR_sat, intR] = step_2dof_pi(rpmR_ref, rpmR, intR, KpR, KiR, KffR, b_2dof, Kb, dz, u_max, Ts_in);
        uR_eff = apply_deadzone(uR_sat, dz);
    end
    
    % =====================================================================
    % TANG 4: TICH PHAN DONG LUC HOC DONG CO & DONG HOC XE (1000 Hz)
    % =====================================================================
    rpmL = adL * rpmL + bdL * uL_eff;
    rpmR = adR * rpmR + bdR * uR_eff;
    
    vR_act = (rpmR * 2 * pi / 60) * r;
    vL_act = (rpmL * 2 * pi / 60) * r;
    v_act  = 0.5 * (vR_act + vL_act);
    w_act  = (vR_act - vL_act) / L;
    
    x     = x + v_act * cos(phi) * Ts_sens;
    y     = y + v_act * sin(phi) * Ts_sens;
    phi   = phi + w_act * Ts_sens;
    s_enc = s_enc + v_act * Ts_sens;
    
    % --- GHI LOG & VE ANIMATION ---
    if mod(step - 1, N_in) == 0
        if log_idx < N_log_max
            log_idx = log_idx + 1;
            log_t(log_idx)        = t;
            log_x(log_idx)        = x;
            log_y(log_idx)        = y;
            log_phi(log_idx)      = phi;
            log_y_active(log_idx) = double(is_y_active);
            if ~isempty(e2_true_ref), log_e2_true(log_idx) = e2_true_ref(1); end
            if ~isempty(e1_filt),     log_e2_filt(log_idx) = e1_filt(1);     end
            if ~isempty(e_lat1_raw),  log_e2_raw(log_idx)  = e_lat1_raw(1);  end
            if ~isempty(e3_est),      log_e3_est(log_idx)  = e3_est(1);      end
            if ~isempty(e3_act),      log_e3_act(log_idx)  = e3_act(1);      end
            if exist('sigma', 'var')
                log_sigma(log_idx)     = sigma;
                log_sat_sigma(log_idx) = sat_sigma;
            end
            log_rpmL(log_idx)     = rpmL;
            log_rpmR(log_idx)     = rpmR;
            log_rpmL_ref(log_idx) = rpmL_ref;
            log_rpmR_ref(log_idx) = rpmR_ref;
        end
        
        if state == 4 && abs(rpmL) < 0.5 && abs(rpmR) < 0.5
            x1_final = x + d1 * cos(phi);
            fprintf('Sai so vi tri dung cuoi duong dan (khong vach ngang): %+.2f mm (Gioi han de bai: +-5 mm)\n', (x1_final - 3.000)*1000);
            break;
        end
        
        if mod(log_idx, anim_skip_log) == 0
            if ~ishandle(fig1), break; end
            xT_hist = (log_x(1:log_idx) + d1 * cos(log_phi(1:log_idx))) * 1000;
            yT_hist = (log_y(1:log_idx) + d1 * sin(log_phi(1:log_idx))) * 1000;
            set(h_traj, 'XData', xT_hist, 'YData', yT_hist);
            update_robot_graphics(x, y, phi, body_len, body_wid, L*1000, wheel_diam, wheel_wid, ...
                d1*1000, h_body, h_wheelL, h_wheelR, h_boom, h_sens1);
            set(h_title, 'String', sprintf('Mo phong Robot (Gauss-Newton 100Hz) | t = %.2f s | e_{2,CB} = %.2f mm | e_{3,uoc} = %.2f deg', ...
                t, e1_filt(1)*1000, rad2deg(e3_est(1))));
            drawnow;
        end
    end
end

idx = 1:log_idx;
log_t = log_t(idx); log_x = log_x(idx); log_y = log_y(idx); log_phi = log_phi(idx);
log_e2_true = log_e2_true(idx); log_e2_filt = log_e2_filt(idx); log_e2_raw = log_e2_raw(idx);
log_e3_est = log_e3_est(idx); log_e3_act = log_e3_act(idx);
log_sigma = log_sigma(idx); log_sat_sigma = log_sat_sigma(idx);
log_rpmL = log_rpmL(idx); log_rpmR = log_rpmR(idx);
log_rpmL_ref = log_rpmL_ref(idx); log_rpmR_ref = log_rpmR_ref(idx);
log_y_active = log_y_active(idx);

if ishandle(fig1)
    xT_hist = (log_x + d1 * cos(log_phi)) * 1000;
    yT_hist = (log_y + d1 * sin(log_phi)) * 1000;
    set(h_traj, 'XData', xT_hist, 'YData', yT_hist);
    update_robot_graphics(x, y, phi, body_len, body_wid, L*1000, wheel_diam, wheel_wid, ...
        d1*1000, h_body, h_wheelL, h_wheelR, h_boom, h_sens1);
    set(h_title, 'String', sprintf('Hoan thanh quy dao bam line (Tai hang: %d kg)', load_kg));
end

delta_e3_deg = rad2deg(log_e3_act - log_e3_est);
valid_steady = log_t > 0.6;
fprintf('\n=== PHAN TICH SAI SO (GAUSS-NEWTON & OBSERVER CUNG CHAY O 100 Hz) ===\n');
fprintf('Sai so lech ngang cuc dai |e2_thuc| (sau 0.6s): %.2f mm (Yeu cau <= 3.0 mm)\n', max(abs(log_e2_true(valid_steady)))*1000);
fprintf('Sai so loc cam bien |e2_loc - e2_thuc| RMS     : %.3f mm\n', rms(log_e2_filt - log_e2_true)*1000);
fprintf('Do lech goc cuc dai |e3_thuc - e3_uoc| (sau 0.6s): %.2f deg (RMS = %.2f deg)\n', ...
    max(abs(delta_e3_deg(valid_steady))), rms(delta_e3_deg(valid_steady)));
fprintf('So lan goi Gauss-Newton tren toan chang       : %d lan (%.3f s)\n', gn_calls_count, sum(log_y_active) * Ts_in);

%% 7. VE CAC FIGURE RIENG BIET
figure('Name', 'Figure 2: Sai so lech ngang e2 (Gauss-Newton 100 Hz)', 'Position', [900, 520, 650, 390]);
hold on; grid on;
plot(log_t, log_e2_raw * 1000, 'Color', [0.78 0.78 0.78], 'LineWidth', 0.9);
plot(log_t, log_e2_true * 1000, 'k-', 'LineWidth', 1.6);
plot(log_t, log_e2_filt * 1000, 'b--', 'LineWidth', 1.5);
plot(log_t, log_y_active * 2.5, 'g-', 'LineWidth', 1.2);
yline(3, 'r--', 'LineWidth', 1.3);
yline(-3, 'r--', 'LineWidth', 1.3);
title('Sai so lech ngang e_2 (mm) - Tach nhanh Gauss-Newton o 100 Hz');
xlabel('Thoi gian (s)'); ylabel('Sai so e_2 (mm)');
ylim([-10 10]);
legend('e_{2,tho} (Cam bien / Tach nhanh)', 'e_{2,thuc} (Hinh hoc chuan)', ...
    'e_{2,loc} (Sau bo loc \alpha-\beta)', 'Co tach nhanh chu Y (2.5 = ON)', ...
    'Gioi han cho phep (\pm3 mm)', 'Location', 'best');

figure('Name', 'Figure 3: So sanh sai so goc e3 thuc te va Bo quan sat Luenberger', 'Position', [900, 60, 650, 390]);
hold on; grid on;
plot(log_t, rad2deg(log_e3_act), 'b-', 'LineWidth', 1.7);
plot(log_t, rad2deg(log_e3_est), 'r--', 'LineWidth', 1.5);
plot(log_t, delta_e3_deg, 'g-', 'LineWidth', 1.4);
yline(0, 'k:', 'LineWidth', 1.0);
title('So sanh sai so goc e_3 thuc te va uoc luong tu Bo quan sat Luenberger (100 Hz)');
xlabel('Thoi gian (s)'); ylabel('Goc (deg)');
legend('e_{3,thuc} (Tiep tuyen quy dao)', 'e_{3,uoc} (Bo quan sat Luenberger)', ...
    '\Deltae_3 = e_{3,thuc} - e_{3,uoc}', 'Location', 'best');

figure('Name', 'Figure 4: Dap ung van toc goc 2 banh xe', 'Position', [120, 60, 720, 390]);
hold on; grid on;
plot(log_t, log_rpmL_ref, 'k--', 'LineWidth', 1.2);
plot(log_t, log_rpmL, 'b-', 'LineWidth', 1.5);
plot(log_t, log_rpmR, 'r-', 'LineWidth', 1.5);
title('Dap ung van toc quay hai banh xe (Vong trong 2-DOF PI)');
xlabel('Thoi gian (s)'); ylabel('Van toc quay (RPM)');
legend('Van toc dat banh trai', 'Van toc thuc banh trai', 'Van toc thuc banh phai', 'Location', 'best');

figure('Name', 'Figure 5: Do thi mat truot sigma & HAM BAO HOA sat(sigma / phi_bl)', 'Position', [120, 520, 720, 380]);
yyaxis left;
plot(log_t, log_sigma * 1000, 'b-', 'LineWidth', 1.6); hold on; grid on;
yline(phi_bl * 1000, 'r--', 'LineWidth', 1.2);
yline(-phi_bl * 1000, 'r--', 'LineWidth', 1.2);
yline(0, 'k:', 'LineWidth', 1.0);
ylabel('Mat truot \sigma (mm)');
ylim([-25 25]);

yyaxis right;
plot(log_t, log_sat_sigma, 'm--', 'LineWidth', 1.3);
ylabel('sat(\sigma / \phi_{bl})');
ylim([-1.3 1.3]);

title('Do thi mat truot \sigma = e_2 + \beta_{smc} e_{3,uoc} + z_{int} va ham bao hoa sat(\sigma / \phi_{bl})');
xlabel('Thoi gian (s)');
legend('\sigma (mm)', '+\phi_{bl} (mm)', '-\phi_{bl} (mm)', 'sat(\sigma / \phi_{bl})', 'Location', 'best');

figure('Name', 'Figure 6: Sai so lech ngang e2 (Khong hien thi co tach nhanh chu Y)', 'Position', [930, 490, 650, 390]);
hold on; grid on;
plot(log_t, log_e2_raw * 1000, 'Color', [0.78 0.78 0.78], 'LineWidth', 0.9);
plot(log_t, log_e2_true * 1000, 'k-', 'LineWidth', 1.6);
plot(log_t, log_e2_filt * 1000, 'b--', 'LineWidth', 1.5);
yline(3, 'r--', 'LineWidth', 1.3);
yline(-3, 'r--', 'LineWidth', 1.3);
title('Sai so lech ngang e_2 (mm) - Gauss-Newton 100 Hz');
xlabel('Thoi gian (s)'); ylabel('Sai so e_2 (mm)');
ylim([-10 10]);
legend('e_{2,tho} (Cam bien / Tach nhanh)', 'e_{2,thuc} (Hinh hoc chuan)', ...
    'e_{2,loc} (Sau bo loc \alpha-\beta)', ...
    'Gioi han cho phep (\pm3 mm)', 'Location', 'best');

%% ================= HAM TANG 1: DOC CAM BIEN 1000 Hz (CHI TINH TRONG TAM) =================
function [e_lat, is_marker, total_I, I_eff] = read_sensor_1000hz(x_row, y_row, phi, map_pts, ...
    y_sens, line_w, marker_x, check_marker, noise_std, ch_gain)

    total_I = 0.0;
    I_eff   = zeros(1, 7);

    nx = -sin(phi); ny = cos(phi);
    sx = x_row + y_sens * nx;
    sy = y_row + y_sens * ny;
    
    mask = abs(map_pts(:,1) - x_row) < 0.08 & abs(map_pts(:,2) - y_row) < 0.08;
    local_pts = map_pts(mask, :);
    if isempty(local_pts)
        e_lat = NaN; is_marker = false; return;
    end
    
    dists = min(hypot(local_pts(:,1) - sx, local_pts(:,2) - sy), [], 1);
    
    delta_opt = 0.0058;
    I_line = 0.5 * (tanh((line_w/2 - dists) / delta_opt) + 1.0);
    
    dx_s = abs(sx - marker_x) - 0.015;
    dy_s = abs(sy - 0.0) - 0.025;
    sd_marker = min(max(dx_s, dy_s), 0.0) + hypot(max(dx_s, 0.0), max(dy_s, 0.0));
    I_marker = 0.5 * (tanh((-sd_marker) / delta_opt) + 1.0);
    
    I_clean = max(I_line, I_marker) .* ch_gain;
    I_noisy = I_clean + noise_std * randn(size(y_sens));
    I_noisy = min(1.0, max(0.0, round(I_noisy * 1023) / 1023));
    
    I_eff = max(0, I_noisy - 0.12);
    I_eff(I_eff < 0.06) = 0;
    
    total_I = sum(I_eff);
    if total_I < 0.25
        e_lat = NaN; is_marker = false; return;
    end
    
    n_active = sum(I_eff > 0.15);
    is_marker = check_marker && (n_active >= 5) && (total_I > 3.40);
    
    e_lat = sum(y_sens .* I_eff) / total_I;
end

%% ================= HAM TANG 2: TACH NHANH GAUSS-NEWTON O 100 Hz =================
function [e_lat, is_y_active] = solve_y_branch_100hz(I_eff, y_sens, total_I, ...
    e_pred, load_kg, line_w, single_line_tot)

    is_y_active   = false;
    delta_opt     = 0.0058;
    inv_delta_opt = 1.0 / delta_opt;
    line_half_w   = line_w * 0.5;

    w_gate = double(abs(y_sens - e_pred) <= 0.021);
    I_gated = I_eff .* w_gate;
    tot_g = sum(I_gated);
    if tot_g < 0.25
        I_gated = I_eff;
        tot_g = total_I;
        w_gate = ones(1, 7);
    end
    
    e_cent = sum(y_sens .* I_gated) / tot_g;
    if tot_g <= single_line_tot
        e_lat = e_cent;
        return;
    end
    
    is_y_active = true;
    branch_sign = (load_kg == 1) * 2.0 - 1.0;
    
    e = e_pred;
    for iter = 1:3
        num   = 0.0;
        denom = 1e-6;
        
        for k = 1:7
            if w_gate(k) == 0.0
                continue;
            end
            
            diff  = y_sens(k) - e;
            dist  = abs(diff);
            u     = (line_half_w - dist) * inv_delta_opt;
            th    = fast_tanh(u);
            I_lin = 0.5 * (th + 1.0);
            
            T0 = I_lin - 0.12;
            if T0 < 0.06
                T0 = 0.0;
                dT = 0.0;
            else
                sgn_diff = (diff >= 0.0) * 2.0 - 1.0;
                dT = 0.5 * (1.0 - th * th) * sgn_diff * inv_delta_opt;
            end
            
            res = I_gated(k) - T0;
            
            w = w_gate(k);
            side_pos = branch_sign * diff;
            if side_pos < -0.006 && res > 0.03
                w = 0.0;
            elseif side_pos < 0.002 && res > 0.04
                w = w * 0.15;
            end
            
            num   = num + w * dT * res;
            denom = denom + w * dT * dT;
        end
        
        step_e = num / denom;
        e = e + max(-0.008, min(0.008, step_e));
    end
    
    alpha_blend = max(0.0, min(1.0, (tot_g - single_line_tot) / 0.16));
    e_lat = (1.0 - alpha_blend) * e_cent + alpha_blend * e;
end

function y = fast_tanh(u)
    if u >= 3.0
        y = 1.0;
    elseif u <= -3.0
        y = -1.0;
    else
        u2 = u * u;
        y  = u * (27.0 + u2) / (27.0 + 9.0 * u2);
    end
end

%% ================= HAM TINH SAI SO HINH HOC CHUAN (CHI DE DOI CHIEU) =================
function [e_lat, e_ang_actual] = calc_exact_errors(x_row, y_row, phi, target_pts)
    e_lat = 0.0;
    e_ang_actual = 0.0;
    dx = target_pts(:, 1) - x_row;
    dy = target_pts(:, 2) - y_row;
    valid_idx = find((dx.^2 + dy.^2) < 0.15^2);
    if isempty(valid_idx)
        return;
    end
    
    dx_loc = dx(valid_idx);
    dy_loc = dy(valid_idx);
    
    x_body =  dx_loc * cos(phi) + dy_loc * sin(phi);
    y_body = -dx_loc * sin(phi) + dy_loc * cos(phi);
    
    [~, min_k] = min(abs(x_body));
    e_lat = y_body(min_k(1));
    
    idx_M = valid_idx(min_k(1));
    i_prev = max(1, idx_M - 2);
    i_next = min(size(target_pts, 1), idx_M + 2);
    tx = target_pts(i_next, 1) - target_pts(i_prev, 1);
    ty = target_pts(i_next, 2) - target_pts(i_prev, 2);
    
    phi_M = atan2(ty, tx);
    e_ang_actual = atan2(sin(phi_M - phi), cos(phi_M - phi));
end

%% ================= CAC HAM PHU TRO KHAC =================
function update_robot_graphics(x, y, phi, b_len, b_wid, track_L, w_diam, w_wid, d1_mm, ...
    h_body, h_wheelL, h_wheelR, h_boom, h_sens1)
    xc = x * 1000; yc = y * 1000;
    c = cos(phi); s = sin(phi);
    to_global_x = @(xl, yl) xc + xl * c - yl * s;
    to_global_y = @(xl, yl) yc + xl * s + yl * c;
    
    bx = [-b_len/2, b_len/2, b_len/2, -b_len/2];
    by = [-b_wid/2, -b_wid/2, b_wid/2, b_wid/2];
    set(h_body, 'XData', to_global_x(bx, by), 'YData', to_global_y(bx, by));
    
    wx = [-w_diam/2, w_diam/2, w_diam/2, -w_diam/2];
    wy_L = [track_L/2 - w_wid/2, track_L/2 - w_wid/2, track_L/2 + w_wid/2, track_L/2 + w_wid/2];
    wy_R = [-track_L/2 - w_wid/2, -track_L/2 - w_wid/2, -track_L/2 + w_wid/2, -track_L/2 + w_wid/2];
    set(h_wheelL, 'XData', to_global_x(wx, wy_L), 'YData', to_global_y(wx, wy_L));
    set(h_wheelR, 'XData', to_global_x(wx, wy_R), 'YData', to_global_y(wx, wy_R));
    
    set(h_boom,  'XData', to_global_x([0, d1_mm], [0, 0]), 'YData', to_global_y([0, d1_mm], [0, 0]));
    set(h_sens1, 'XData', to_global_x([d1_mm, d1_mm], [-27, 27]), 'YData', to_global_y([d1_mm, d1_mm], [-27, 27]));
end

function [u_sat, integ_new] = step_2dof_pi(ref, meas, integ, Kp, Ki, Kff, b, Kb, dz, u_max, Ts)
    err = ref - meas;
    u_ff = Kff * ref;
    u_p  = Kp * (b * ref - meas);
    u_unsat = u_ff + u_p + integ;
    
    if abs(u_unsat) > 1e-4
        u_cmd = u_unsat + dz * sign(u_unsat);
    else
        u_cmd = 0;
    end
    
    u_sat = max(-u_max, min(u_max, u_cmd));
    integ_new = integ + (Ki * err + Kb * (u_sat - u_cmd)) * Ts;
end

function u_eff = apply_deadzone(u_in, dz)
    if abs(u_in) <= dz
        u_eff = 0;
    else
        u_eff = u_in - dz * sign(u_in);
    end
end

function [common_pts, branch1_pts, branch2_pts, marker_x] = build_track_map()
    marker_x = 1.000;
    
    R_s     = 0.800;
    alpha_s = pi / 4;
    T_s     = R_s * tan(alpha_s / 2);
    
    x_t1 = 1.000 - T_s;
    x_t2 = 1.500 + T_s;
    
    s1 = [linspace(3.05, 0.500, 4050)', zeros(4050, 1)];
    th_u = linspace(-pi/2, -3*pi/2, 2500)';
    arc_u = [0.500 + 0.500*cos(th_u), 0.500 + 0.500*sin(th_u)];
    s_shared = [linspace(0.500, x_t1, 400)', 1.000*ones(400, 1)];
    common_pts = [s1; arc_u; s_shared];
    
    th1a  = linspace(-pi/2, -pi/2 + alpha_s, 1000)';
    arc1a = [x_t1 + R_s*cos(th1a), (1.000 + R_s) + R_s*sin(th1a)];
    th1b  = linspace(pi/2 + alpha_s, pi/2, 1000)';
    arc1b = [x_t2 + R_s*cos(th1b), (1.500 - R_s) + R_s*sin(th1b)];
    diag1 = [linspace(arc1a(end,1), arc1b(1,1), 200)', linspace(arc1a(end,2), arc1b(1,2), 200)'];
    str1  = [linspace(x_t2, 3.000, 1800)', 1.500*ones(1800, 1)];
    branch1_pts = [arc1a; diag1; arc1b; str1];
    
    th2a  = linspace(pi/2, pi/2 - alpha_s, 1000)';
    arc2a = [x_t1 + R_s*cos(th2a), (1.000 - R_s) + R_s*sin(th2a)];
    th2b  = linspace(-pi/2 - alpha_s, -pi/2, 1000)';
    arc2b = [x_t2 + R_s*cos(th2b), (0.500 + R_s) + R_s*sin(th2b)];
    diag2 = [linspace(arc2a(end,1), arc2b(1,1), 200)', linspace(arc2a(end,2), arc2b(1,2), 200)'];
    str2  = [linspace(x_t2, 3.000, 1800)', 0.500*ones(1800, 1)];
    branch2_pts = [arc2a; diag2; arc2b; str2];
end

function v_out = calc_decel_velocity(d_rem, v_cruise, v_min, D_dec, d_margin, a_dec)
    if d_rem > D_dec
        v_out = v_cruise;
    else
        d_eff = max(0, d_rem - d_margin);
        v_out = min(v_cruise, sqrt(v_min^2 + 2 * a_dec * d_eff));
    end
end