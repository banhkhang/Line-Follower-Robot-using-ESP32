% =========================================================================
%  XuatSoDoToanHeThong.m
%  Xuất sơ đồ khối CHU TRÌNH TÍN HIỆU TOÀN HỆ THỐNG (Đa tần số, 2 nhân ESP32,
%  Hệ A Live Wi-Fi / Hệ B SRAM Burst, Vòng ngoài SMC + Luenberger + Gauss-Newton,
%  Vòng trong 2-DOF PI + M/T 1x, Loadcell HX711 + Lập lịch tham số).
%
%  Phong cách đồng bộ với XuatHinhAnhBaoCao.m (Times New Roman, vector SVG).
%  Không yêu cầu bất kỳ Toolbox nào ngoài MATLAB chuẩn.
%
%  Đầu ra trong thư mục figures/:
%   1) Hinh_TongThe_ChuTrinhTinHieu_Mau.svg       (Bản màu học thuật - Vector)
%   2) Hinh_TongThe_ChuTrinhTinHieu_Mau.png       (Bản màu 300 DPI)
%   3) Hinh_TongThe_ChuTrinhTinHieu_TrangDen.svg  (Bản trắng-đen in ấn - Vector)
%   4) Hinh_TongThe_ChuTrinhTinHieu_TrangDen.png  (Bản trắng-đen 300 DPI)
% =========================================================================
clear; clc; close all;

out_dir = fullfile(pwd, 'figures');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultFigureColor',  'w');

% Xuất cả 2 phiên bản: Bản màu học thuật & Bản trắng-đen chuẩn in ấn báo cáo
render_full_system_diagram(false, out_dir);
render_full_system_diagram(true,  out_dir);

fprintf('\nHOÀN TẤT! Đã xuất đầy đủ sơ đồ toàn hệ thống (SVG + PNG 300 DPI) vào figures/.\n');


%% =========================================================================
%  HÀM CHÍNH VẼ SƠ ĐỒ TOÀN HỆ THỐNG (KHÔNG GIAN TỌA ĐỘ 470 x 312 mm)
% =========================================================================
function render_full_system_diagram(bw_mode, out_dir)

    % --- Bảng màu (Chế độ màu học thuật vs Trắng-Đen chuẩn in ấn) ---
    if bw_mode
        C_BLACK  = [0.08, 0.08, 0.08];
        C_BLUE   = [0.12, 0.12, 0.12];
        C_RED    = [0.15, 0.15, 0.15];
        C_GREEN  = [0.12, 0.12, 0.12];
        C_ORANGE = [0.20, 0.20, 0.20];
        C_PURPLE = [0.15, 0.15, 0.15];
        C_TEAL   = [0.12, 0.12, 0.12];
        C_GRAY   = [0.40, 0.40, 0.40];

        Z_COM_BG = [0.97, 0.97, 0.97];
        Z_LD_BG  = [0.97, 0.97, 0.97];
        Z_OUT_BG = [0.98, 0.98, 0.98];
        Z_IN_BG  = [0.96, 0.96, 0.96];
        Z_PHY_BG = [0.97, 0.97, 0.97];
        Z_SEN_BG = [0.98, 0.98, 0.98];
        BOX_BG   = [1.00, 1.00, 1.00];
        BOX_HI   = [0.94, 0.94, 0.94];
    else
        C_BLACK  = [0.10, 0.10, 0.12];
        C_BLUE   = [0.00, 0.30, 0.65];   % Vòng ngoài / Kênh A / Core 0
        C_RED    = [0.76, 0.12, 0.12];   % Kênh B / Watchdog / Adaptive Trigger
        C_GREEN  = [0.00, 0.46, 0.32];   % Chuỗi cảm biến / Ước lượng
        C_ORANGE = [0.80, 0.38, 0.00];   % Loadcell / Lập lịch tham số
        C_PURPLE = [0.42, 0.16, 0.60];   % MATLAB / Wi-Fi / SRAM / NVS
        C_TEAL   = [0.00, 0.45, 0.55];   % Vòng trong Core 1 / Odometry
        C_GRAY   = [0.40, 0.42, 0.46];

        Z_COM_BG = [0.96, 0.94, 0.99];
        Z_LD_BG  = [1.00, 0.96, 0.91];
        Z_OUT_BG = [0.93, 0.96, 1.00];
        Z_IN_BG  = [0.92, 0.98, 0.98];
        Z_PHY_BG = [0.96, 0.96, 0.97];
        Z_SEN_BG = [0.92, 0.98, 0.95];
        BOX_BG   = [1.00, 1.00, 1.00];
        BOX_HI   = [0.98, 0.99, 1.00];
    end

    % Khổ lớn tỉ lệ A3 ngang (47 cm x 31.2 cm) -> chữ sắc nét tuyệt đối khi chèn báo cáo
    fig = figure('Units', 'centimeters', 'Position', [0.5 0.5 47.0 31.2], 'Visible', 'off');
    ax  = axes(fig, 'Position', [0.006 0.006 0.988 0.988]);
    hold(ax, 'on');
    xlim(ax, [0 470]); ylim(ax, [0 312]);
    daspect(ax, [1 1 1]); axis(ax, 'off');

    % =====================================================================
    % TIÊU ĐỀ TỔNG THỂ (y = 296..311)
    % =====================================================================
    text(ax, 235, 307.2, ...
        'KIẾN TRÚC ĐIỀU KHIỂN PHÂN TẦNG ĐA TẦN SỐ & CHU TRÌNH TÍN HIỆU TOÀN HỆ THỐNG ROBOT BÁM LINE', ...
        'HorizontalAlignment', 'center', 'FontSize', 13.8, 'FontWeight', 'bold', 'Color', C_BLACK);
    text(ax, 235, 300.6, ...
        'ESP32-WROOM-32E (Core 0 / Core 1)  |  Hệ A (5 mắt + Wi-Fi Live 100 Hz) & Hệ B (7 mắt + SRAM Burst)  |  B = 0,180 m, d_1 = 0,055 m, D = 0,065 m, v_{req} = 0,50 m/s', ...
        'HorizontalAlignment', 'center', 'FontSize', 9.2, 'FontAngle', 'italic', 'Color', C_GRAY);

    % =====================================================================
    % 1. VẼ 6 KHUNG PHÂN VÙNG KIẾN TRÚC (CÓ HÀNH LANG BUS RỘNG 18 mm GIỮA CÁC TẦNG)
    % =====================================================================
    % Tầng trên: y = 234..288 (Hành lang trên cùng cho m_load: y = 288..296)
    draw_zone(ax, 6,   234, 200, 54, Z_COM_BG, C_PURPLE, ...
        'VÙNG 1: GIÁM SÁT MATLAB, ĐIỀU PHỐI CHẾ ĐỘ (CORE 0), SRAM BURST & NVS');
    draw_zone(ax, 218, 234, 168, 54, Z_LD_BG,  C_ORANGE, ...
        'VÙNG 2: ĐO TẢI TRỌNG HX711 (10 SPS) & BỘ LUẬT LẬP LỊCH THAM SỐ');

    % Hành lang giữa Tầng 1 và Tầng 2: y = 216..234 (cao 18 mm -> 0 chồng chữ!)
    % Tầng giữa: y = 94..216 (cao 122 mm)
    draw_zone(ax, 6,   94, 200, 122, Z_OUT_BG, C_BLUE, ...
        'VÙNG 3: VÒNG ĐIỀU KHIỂN BÁM LINE BÊN NGOÀI — CORE 0 (200 Hz \rightarrow 100 Hz \rightarrow SMC 50–160 ms)');
    draw_zone(ax, 218, 94, 168, 122, Z_IN_BG,  C_TEAL, ...
        'VÙNG 4: VÒNG ĐIỀU KHIỂN VẬN TỐC BÊN TRONG — CORE 1 (GPTIMER 200 Hz, T_s = 5 ms)');

    % Vùng 5 (Phải): y = 94..288 (chạy dọc toàn bộ 2 tầng trên & giữa)
    draw_zone(ax, 396, 94, 68, 194, Z_PHY_BG, C_BLACK, ...
        'VÙNG 5: ĐỐI TƯỢNG VẬT LÝ');

    % Hành lang giữa Tầng 2 và Tầng 3: y = 70..94 (cao 24 mm cho 3 tuyến bus độc lập!)
    % Tầng dưới (Vùng 6): y = 8..70 (cao 62 mm)
    draw_zone(ax, 6, 8, 458, 62, Z_SEN_BG, C_GREEN, ...
        'VÙNG 6: CHUỖI ĐO QUANG HỌC TCRT5000 (200 Hz), TÁCH NHÁNH Y GAUSS-NEWTON (100 Hz) & BỘ QUAN SÁT LUENBERGER THÍCH NGHI (100 Hz)');

    % =====================================================================
    % 2. VÙNG 1: MATLAB — APP/WIFI/OTA — SRAM LOGGER & NVS (y = 239..280)
    % =====================================================================
    draw_box(ax, 11, 239, 54, 41, BOX_BG, C_PURPLE, 1.25, { ...
        '\bfMáy tính MATLAB (PC)', ...
        'robot\_control\_console.m', ...
        '• Lệnh: PING, START\_A/B,', ...
        '  STOP, SET/GET\_PARAMS', ...
        '• Nhận Live (Hệ A) & Burst', ...
        '  chống mất gói (Hệ B)', ...
        '• Lưu .mat/.csv + 5 đồ thị'}, 6.9, C_BLACK);

    draw_box(ax, 81, 239, 62, 41, BOX_BG, C_PURPLE, 1.25, { ...
        '\bfĐiều phối & Wi-Fi SoftAP', ...
        'app.c + matlab\_link.c + ota.c', ...
        '• AP: LineRobot-AP (192.168.4.1)', ...
        '• BOOT (IO0): 1x(7S), 2x(5S),', ...
        '  giữ 2s \rightarrow Calib (LED 1s/1s\times3)', ...
        '• Chuyển profile động tại runtime:', ...
        '  Wi-Fi ON \Leftrightarrow 5S | OFF \Leftrightarrow 7S'}, 6.9, C_BLACK);

    draw_box(ax, 154, 239, 47, 41, BOX_BG, C_PURPLE, 1.25, { ...
        '\bfSRAM Logger & NVS', ...
        'data\_logger.c (Hệ B)', ...
        '• Chunk 200\times52B (tối đa', ...
        '  2000 mẫu = 104 KB)', ...
        '• Giữ \geq88 KB cho Wi-Fi', ...
        '• Tóm tắt 44B (KPI, lỗi)', ...
        '• NVS: R_w[7], R_b[7]'}, 6.8, C_BLACK);

    % Kết nối nội bộ Vùng 1
    draw_arrow(ax, 65, 265, 81, 265, C_PURPLE, 1.15, '-');
    text(ax, 73, 268.8, 'UDP :8889', 'HorizontalAlignment', 'center', 'FontSize', 6.7, 'FontWeight', 'bold', 'Color', C_PURPLE);
    draw_arrow(ax, 81, 250, 65, 250, C_PURPLE, 1.15, '-');
    text(ax, 73, 246.2, 'UDP :8888', 'HorizontalAlignment', 'center', 'FontSize', 6.7, 'FontWeight', 'bold', 'Color', C_PURPLE);
    text(ax, 73, 241.6, 'Live/Burst', 'HorizontalAlignment', 'center', 'FontSize', 6.2, 'Color', C_PURPLE);

    draw_double_arrow(ax, 143, 259, 154, 259, C_PURPLE, 1.1, '-');
    text(ax, 148.5, 266.2, {'52B/mẫu', 'Chunk'}, 'HorizontalAlignment', 'center', 'FontSize', 6.2, 'Color', C_PURPLE);

    % =====================================================================
    % 3. VÙNG 2: LOADCELL HX711 & LẬP LỊCH THAM SỐ (y = 239..280)
    % =====================================================================
    draw_box(ax, 223, 240, 42, 39, BOX_BG, C_ORANGE, 1.25, { ...
        '\bfCảm biến tải', ...
        '\bfLoadcell + HX711', ...
        '• DOUT (IO14)', ...
        '• PD\_SCK (IO27)', ...
        '• Gain 128, 10 SPS', ...
        '• 24-bit ADC thô'}, 6.9, C_BLACK);

    draw_box(ax, 276, 240, 48, 39, BOX_BG, C_ORANGE, 1.25, { ...
        '\bfXử lý & Phân loại tải', ...
        'loadcell\_service.c', ...
        '• net = raw - offset_{tare}', ...
        '• m_g = |net / 388,7| (ABS)', ...
        '• Khóa: >700g, \pm50g / 1,2s', ...
        '• <1,5kg \rightarrow 1kg | \geq1,5 \rightarrow 2kg'}, 6.8, C_BLACK);

    draw_box(ax, 334, 240, 47, 39, BOX_BG, C_ORANGE, 1.25, { ...
        '\bfBộ luật Tham số', ...
        'vel\_ctrl\_apply\_payload', ...
        '• K_{p,A} = 291 + 67,6m', ...
        '• K_{p,B} = 326,7 + 79,7m', ...
        '• u_{dz,A} = 4,51 + 2,78m', ...
        '• u_{dz,B} = 7,32 + 5,21m', ...
        '• e_{band} = 0,020 + 0,005m'}, 6.6, C_BLACK);

    draw_arrow(ax, 265, 259.5, 276, 259.5, C_ORANGE, 1.15, '-');
    text(ax, 270.5, 263.5, 'V_{load}', 'HorizontalAlignment', 'center', 'FontSize', 6.9, 'FontWeight', 'bold', 'Color', C_ORANGE);
    draw_arrow(ax, 324, 259.5, 334, 259.5, C_ORANGE, 1.15, '-');
    text(ax, 329.0, 263.5, 'm_{đo}', 'HorizontalAlignment', 'center', 'FontSize', 6.9, 'FontWeight', 'bold', 'Color', C_ORANGE);

    % Khối hàng m_load nằm trên hành lang y = 292.5 (HOÀN TOÀN TRÊN Zone 2 & Zone 5!)
    plot(ax, [208 244], [292.5 292.5], '-', 'Color', C_ORANGE, 'LineWidth', 1.25);
    text(ax, 224, 295.3, 'Khối hàng m_{load} \in \{0, 1, 2\} kg', 'HorizontalAlignment', 'center', ...
        'FontSize', 7.4, 'FontWeight', 'bold', 'Color', C_ORANGE);
    draw_dot(ax, 244, 292.5, C_ORANGE);
    draw_arrow(ax, 244, 292.5, 244, 279, C_ORANGE, 1.15, '-');
    draw_polyline_arrow(ax, [244 292.5; 430 292.5; 430 278], C_ORANGE, 1.15, '-');

    % =====================================================================
    % 4. VÙNG 3: VÒNG NGOÀI CORE 0 (y = 98..208)
    % =====================================================================
    % 3A: Mission FSM & Watchdogs (y = 158..206)
    draw_box(ax, 14, 158, 108, 48, BOX_BG, C_BLUE, 1.3, { ...
        '\bfMáy Trạng thái Nhiệm vụ & Watchdog An toàn (mission\_ctrl.c)', ...
        '• Chuỗi: TO\_LOAD \rightarrow LOAD\_AXLE\_ALIGN (+d_1) \rightarrow WAIT\_LOAD (HX711)', ...
        '  \rightarrow TO\_GOAL (Y-zone) \rightarrow GOAL\_AXLE\_ALIGN (+d_1) \rightarrow STOPPED\_GOAL (2s)', ...
        '• Vạch ngang: n_{act}\geq4/5, ΣI>2,65/3,20 | Hết vạch EOL: s\geq4,45m, ΣI<1,50/1,65', ...
        '• Căn dừng TRỤC BÁNH TRƯỚC: s_{stop} = s_{mốc} + d_1 + trim - d_{brake}(v, m)', ...
        '• 3 điều kiện [Stop]: Mất line \geq2s | |e_2|>16mm \geq2s | Dừng đích 2s \rightarrow Burst'}, 6.7, C_BLACK);

    % 3B: Velocity Profiler (y = 158..206)
    draw_box(ax, 134, 158, 66, 48, BOX_BG, C_BLUE, 1.3, { ...
        '\bfQuy hoạch Vận tốc dài', ...
        '(outer\_smc.c — 100 Hz)', ...
        '• Giảm tốc về v_{min}=0,10 m/s:', ...
        '  v_{prof} = √(v_{min}^2 + 2a_{dec}\Deltad)', ...
        '• Nhịp giảm tốc cosine ngã ba Y:', ...
        '  v_{junc} \in [0,32..0,50] m/s', ...
        '• v_{cmd} = min(v_{prof}, v_{junc})', ...
        '• Bò thẳng EOL: v_{min}, \omega_{cmd}=0'}, 6.7, C_BLACK);

    % 3C: Adaptive SMC Scheduler (y = 99..145)
    draw_box(ax, 14, 99, 74, 46, BOX_BG, C_RED, 1.25, { ...
        '\bfLập lịch Chu kỳ SMC Thích nghi', ...
        '(outer\_smc\_step\_adaptive, 100 Hz)', ...
        '• Kiểm tra xác lập vòng trong 2 bánh:', ...
        '  |v_{ref,i}-v_{act,i}| \leq max(0,02; 5%v_{ref})', ...
        '• Giữ ổn định t_{dwell} \geq 20 ms & \Deltat \geq T_{s,min}', ...
        '  HOẶC chạm trần \Deltat \geq T_{s,max} (< 200 ms)', ...
        '• Phát xung kích & \Deltat_{SMC} \in [50..160] ms'}, 6.6, C_BLACK);

    % 3D: Integral SMC Controller (y = 99..145)
    draw_box(ax, 102, 99, 98, 46, BOX_HI, C_BLUE, 1.4, { ...
        '\bfBộ Điều khiển Trượt Tích phân — Integral SMC', ...
        '(Doria-Cerezo et al., ECC 2019 | outer\_smc.c)', ...
        '• z_{int} \leftarrow clip(z_{int} + k_i\cdot e_{2,filt}\cdot \Deltat_{SMC}, \pm z_{max})  (t > 0,3s)', ...
        '• Mặt trượt: \sigma = e_{2,filt} + \beta\cdot ê_3 + z_{int}   |   sat(\sigma / \phi_{bl})', ...
        '• u_{eq} = [v_{cmd}(sin(ê_3) + \beta\cdot ĉ) + k_i\cdot e_{2,filt}] / (d_1 + \beta)', ...
        '• \omega_{cmd} = clip(u_{eq} + [k_{smc}/(d_1+\beta)]\cdot sat(\sigma/\phi_{bl}), \pm\omega_{max})'}, 6.7, C_BLACK);

    % Kết nối nội bộ Vùng 3 (HOÀN TOÀN KHÔNG ĐÈ NHAU)
    draw_arrow(ax, 122, 182, 134, 182, C_BLUE, 1.15, '-');
    text(ax, 128, 188.5, {'state,', 'd_{rem}'}, 'HorizontalAlignment', 'center', 'FontSize', 6.4, 'Color', C_BLUE);

    draw_arrow(ax, 88, 122, 102, 122, C_RED, 1.2, '--');
    text(ax, 95, 129.0, {'trig,', '\Deltat_{SMC}'}, 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'FontWeight', 'bold', 'Color', C_RED);

    % v_cmd từ 3B xuống 3D tại x = 167 (nhãn nằm phía trên đường Brake/S-c)
    draw_arrow(ax, 167, 158, 167, 145, C_BLUE, 1.15, '-');
    text(ax, 170.5, 154.8, 'v_{cmd}', 'HorizontalAlignment', 'left', 'FontSize', 6.8, ...
        'FontWeight', 'bold', 'Color', C_BLUE, 'BackgroundColor', Z_OUT_BG, 'Margin', 0.2);

    % =====================================================================
    % 5. VÙNG 4: VÒNG TRONG CORE 1 (y = 98..208)
    % =====================================================================
    % 4A: S-curve & Inverse Differential Kinematics
    draw_box(ax, 223, 144, 35, 44, BOX_BG, C_TEAL, 1.25, { ...
        '\bfS-Curve bậc 3 &', ...
        '\bfĐộng học nghịch', ...
        'drive\_service.c', ...
        'v(\tau)=v_0+\Deltav(3\tau^2-2\tau^3)', ...
        'a_{max}=0,9375 m/s^2', ...
        'v_A=v_{eff}-(B/2)\omega', ...
        'v_B=v_{eff}+(B/2)\omega'}, 6.3, C_BLACK);

    % KÊNH A (Bánh TRÁI — y = 162..206)
    draw_sum_x(ax, 272, 195, 3.0, C_BLUE);
    text(ax, 267.4, 197.6, '+', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', C_BLUE);
    text(ax, 274.4, 190.2, '–', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C_BLUE);

    draw_box(ax, 281, 183, 44, 24, BOX_BG, C_BLUE, 1.25, { ...
        '\bf2-DOF PI (Kênh A)', ...
        'u_{ff}=K_{ff,A}v+u_{dz,A}', ...
        'Tustin I_A (|e_A|<e_{band})', ...
        'Anti-Plug / Short Brake'}, 6.3, C_BLACK);

    draw_box(ax, 334, 183, 47, 24, BOX_BG, C_BLUE, 1.25, { ...
        '\bf6N137 + DRV8871 (A)', ...
        '\bf& Động cơ A (Trái)', ...
        'PWM 10kHz 12-bit (IO12/13)', ...
        'JGB37-520 (30:1)'}, 6.3, C_BLACK);

    draw_box(ax, 296, 161, 68, 15, BOX_BG, C_BLUE, 1.1, { ...
        '\bfEncoder Hall A (IO21/19) — M/T 1x IRAM ISR', ...
        'N_{1x}=330, lọc EMI<80\mus, chặn gai 30RPM \rightarrow v_{A,đo}, N_A'}, 6.2, C_BLACK);

    % KÊNH B (Bánh PHẢI — y = 101..146)
    draw_sum_x(ax, 272, 138, 3.0, C_RED);
    text(ax, 267.4, 140.6, '+', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', C_RED);
    text(ax, 274.4, 133.2, '–', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C_RED);

    draw_box(ax, 281, 126, 44, 24, BOX_BG, C_RED, 1.25, { ...
        '\bf2-DOF PI (Kênh B)', ...
        'u_{ff}=K_{ff,B}v+u_{dz,B}', ...
        'Tustin I_B (|e_B|<e_{band})', ...
        'Anti-Plug / Short Brake'}, 6.3, C_BLACK);

    draw_box(ax, 334, 126, 47, 24, BOX_BG, C_RED, 1.25, { ...
        '\bf6N137 + DRV8871 (B)', ...
        '\bf& Động cơ B (Phải)', ...
        'PWM 10kHz 12-bit (IO16/15)', ...
        'JGB37-520 (30:1)'}, 6.3, C_BLACK);

    draw_box(ax, 296, 102, 68, 15, BOX_BG, C_RED, 1.1, { ...
        '\bfEncoder Hall B (IO18/17) — M/T 1x IRAM ISR', ...
        'N_{1x}=330, \Deltat_{\mus} (1\mus), dir=+1 \rightarrow v_{B,đo}, N_B'}, 6.2, C_BLACK);

    % 4D: Forward Kinematics, Odometry & 200 Hz Sync
    draw_box(ax, 223, 99, 61, 29, BOX_HI, C_TEAL, 1.25, { ...
        '\bfĐộng học thuận & Odometry', ...
        '• v_{act} = 0,5(v_A + v_B)', ...
        '• \omega_{act} = (v_B - v_A) / B', ...
        '• s_{enc} = 0,5(\DeltaN_A+\DeltaN_B)\cdot C/330', ...
        '• Notify 200Hz (jitter < 2\mus)'}, 6.4, C_BLACK);

    % Nối bên trong Vùng 4 (Kênh A)
    draw_polyline_arrow(ax, [258 176; 263 176; 263 195; 269 195], C_BLUE, 1.1, '-');
    text(ax, 259, 199.0, 'v_{ref,A}', 'HorizontalAlignment', 'center', 'FontSize', 6.7, 'FontWeight', 'bold', 'Color', C_BLUE);
    draw_arrow(ax, 275, 195, 281, 195, C_BLUE, 1.1, '-');
    text(ax, 278, 198.2, 'e_A', 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'Color', C_BLUE);
    draw_arrow(ax, 325, 195, 334, 195, C_BLUE, 1.1, '-');
    text(ax, 329.5, 198.2, 'u_A%', 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'FontWeight', 'bold', 'Color', C_BLUE);
    draw_arrow(ax, 381, 195, 401, 195, C_BLUE, 1.25, '-');
    text(ax, 391, 198.8, 'v_A', 'HorizontalAlignment', 'center', 'FontSize', 7.3, 'FontWeight', 'bold', 'Color', C_BLUE);

    % Hồi tiếp Encoder A
    draw_dot(ax, 384, 195, C_BLUE);
    draw_polyline_arrow(ax, [384 195; 384 168.5; 364 168.5], C_BLUE, 1.05, '-');
    draw_polyline_arrow(ax, [296 168.5; 272 168.5; 272 192], C_BLUE, 1.05, '-');
    text(ax, 285, 171.8, 'v_{A,đo}', 'HorizontalAlignment', 'center', 'FontSize', 6.6, 'Color', C_BLUE);
    draw_dot(ax, 277, 168.5, C_BLUE);
    draw_polyline_arrow(ax, [277 168.5; 277 156; 262 156; 262 128], C_TEAL, 0.95, ':');

    % Nối bên trong Vùng 4 (Kênh B)
    draw_polyline_arrow(ax, [258 158; 263 158; 263 138; 269 138], C_RED, 1.1, '-');
    text(ax, 257, 133.5, 'v_{ref,B}', 'HorizontalAlignment', 'center', 'FontSize', 6.7, 'FontWeight', 'bold', 'Color', C_RED);
    draw_arrow(ax, 275, 138, 281, 138, C_RED, 1.1, '-');
    text(ax, 278, 141.2, 'e_B', 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'Color', C_RED);
    draw_arrow(ax, 325, 138, 334, 138, C_RED, 1.1, '-');
    text(ax, 329.5, 141.2, 'u_B%', 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'FontWeight', 'bold', 'Color', C_RED);
    draw_arrow(ax, 381, 138, 401, 138, C_RED, 1.25, '-');
    text(ax, 391, 141.8, 'v_B', 'HorizontalAlignment', 'center', 'FontSize', 7.3, 'FontWeight', 'bold', 'Color', C_RED);

    % Hồi tiếp Encoder B
    draw_dot(ax, 384, 138, C_RED);
    draw_polyline_arrow(ax, [384 138; 384 109.5; 364 109.5], C_RED, 1.05, '-');
    draw_polyline_arrow(ax, [296 109.5; 272 109.5; 272 135], C_RED, 1.05, '-');
    text(ax, 288.5, 105.5, 'v_{B,đo}', 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'Color', C_RED);
    draw_dot(ax, 288, 109.5, C_RED);
    draw_arrow(ax, 288, 109.5, 284, 109.5, C_TEAL, 0.95, ':');

    % =====================================================================
    % 6. VÙNG 5: ĐỐI TƯỢNG VẬT LÝ (ROBOT VI SAI & SA BÀN ĐỀ BÀI)
    % =====================================================================
    draw_box(ax, 401, 218, 58, 60, BOX_BG, C_BLACK, 1.2, { ...
        '\bfMôi trường Sa bàn', ...
        '• Nền trắng, line đen 26mm', ...
        '• Chặng 1: 2,000 m', ...
        '  Vạch ngang 26\times50 mm', ...
        '• Chặng 2: U-turn R500', ...
        '  + Ngã ba Y (R800):', ...
        '  1kg Trái | 2kg Phải', ...
        '• Đích: 4,709m (không vạch)', ...
        '• Chỉ tiêu: \pm3mm / \pm5mm'}, 6.6, C_BLACK);

    draw_box(ax, 401, 104, 58, 104, BOX_HI, C_BLACK, 1.4, { ...
        '\bfRobot Vi sai 2 Bánh', ...
        '\bftrên Mặt bạt Sa bàn', ...
        '• Dẫn động trước (B=0,18m)', ...
        '  Bánh tự do sau (152 mm)', ...
        '• d_1 = 55 mm trước trục', ...
        '• m_{xe}=1,3kg + m_{load}', ...
        '  CoG: 37 / 49 / 57 mm', ...
        '• Động học sai số (PT 5):', ...
        '  de_2/dt = v\cdot sin(e_3) - d_1\omega', ...
        '  de_3/dt = c(s)\cdot v - \omega', ...
        '• Vị trí dãy cảm biến:', ...
        '  x_1 = x + d_1 cos(\phi)', ...
        '  y_1 = y + d_1 sin(\phi)'}, 6.6, C_BLACK);

    draw_arrow(ax, 430, 218, 430, 208, C_BLACK, 1.05, '--');
    text(ax, 444.5, 213.0, 'c(s), vạch', 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'Color', C_GRAY);

    % =====================================================================
    % 7. VÙNG 6: CHUỖI CẢM BIẾN QUANG HỌC, LỌC & ƯỚC LƯỢNG (y = 12..61)
    % =====================================================================
    draw_box(ax, 380, 12, 78, 49, BOX_BG, C_GREEN, 1.25, { ...
        '\bfDãy 7\times TCRT5000 + MOSFET IR', ...
        '(tcrt5000.c — 200 Hz, 0 ngắt phụ)', ...
        '• S1..S6 (ADC1: IO36,39,34,35,32,33)', ...
        '• S7 (ADC2: IO25) | CTRL (IO26)', ...
        '• Khử nền: V_{net} = V_{off} - V_{on} (80\mus)', ...
        '• Oversample 2\times liên tiếp (~0,8 ms)', ...
        '• Profile: 5S (Wi-Fi ON) | 7S (OFF)'}, 6.6, C_BLACK);

    draw_box(ax, 287, 12, 79, 49, BOX_BG, C_GREEN, 1.25, { ...
        '\bfChuẩn hóa, Tuyến tính hóa 5S', ...
        '\bf& Đổi hệ trục (sensor\_service.c)', ...
        '• norm_i = (R_{w,i}-V_{net,i})/(R_{w,i}-R_{b,i})', ...
        '• I_{eff,i} = max(0, norm_i/100 - 0,12)', ...
        '• Bù biên 5S (|pos|>7,44mm, K=1,5132)', ...
        '• Đổi dấu PCB (S1=-27) \rightarrow ISO/SMC:', ...
        '  e_{2,raw} = -pos\cdot 10^{-3} (+ = Trái)'}, 6.6, C_BLACK);

    draw_box(ax, 196, 12, 77, 49, BOX_BG, C_GREEN, 1.25, { ...
        '\bfTách nhánh chữ Y Gauss-Newton', ...
        '(outer\_estimator.c — 100 Hz)', ...
        '• Cửa sổ: s_{enc} \in [2,05..2,70] m', ...
        '• Kích hoạt khi ΣI_g > 2,36(7S)/2,20(5S)', ...
        '• Gating |y_k - e_{pred}|\leq21mm, 3 vòng', ...
        '• Mẫu tanh Padé [3/2], trọng số bất đối', ...
        '  xứng: 1kg (+1 Trái) | 2kg (-1 Phải)'}, 6.6, C_BLACK);

    draw_box(ax, 105, 12, 77, 49, BOX_BG, C_GREEN, 1.25, { ...
        '\bfBộ lọc Tiền xử lý Alpha-Beta', ...
        '(outer\_estimator.c — 200 / 100 Hz)', ...
        '• \omega_{n,f} = 75 rad/s, \zeta_f = 0,95', ...
        '• Không trễ pha xác lập (bậc 2)', ...
        '• e_{2,pred} = e_{2,filt} + v_{e2,filt}\cdot T_s', ...
        '• Ngoài Y: cập nhật 200 Hz từ e_{2,raw}', ...
        '• Trong Y: cập nhật 100 Hz từ e_{2,Y}'}, 6.6, C_BLACK);

    draw_box(ax, 14, 12, 77, 49, BOX_HI, C_GREEN, 1.35, { ...
        '\bfBộ Quan sát Luenberger Thích nghi', ...
        '(outer\_estimator.c — 100 Hz, T_s=10ms)', ...
        '• Độ cong: ĉ = LPF(\omega_{act}/max(|v|,0,3), 35ms)', ...
        '• Trộn cực theo |e_{2,filt} - ê_2|:', ...
        '  slow(0,89;0,92) \leftrightarrow fast(0,75;0,80)', ...
        '• Ackermann (L_{d1}, L_{d2}) + Taylor bậc 2', ...
        '• Xuất ước lượng: ê_2, ê_3 [rad], ĉ [1/m]'}, 6.6, C_BLACK);

    % Nối chuỗi ngang trong Vùng 6 (Phải -> Trái)
    draw_arrow(ax, 380, 36.5, 366, 36.5, C_GREEN, 1.2, '-');
    text(ax, 373, 41.8, {'V_{net}', '[1..7]'}, 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'FontWeight', 'bold', 'Color', C_GREEN);

    draw_arrow(ax, 287, 36.5, 273, 36.5, C_GREEN, 1.2, '-');
    text(ax, 280, 42.2, {'I_{eff}[7],', 'ΣI, e_{2,raw}'}, 'HorizontalAlignment', 'center', 'FontSize', 6.3, 'FontWeight', 'bold', 'Color', C_GREEN);

    draw_arrow(ax, 196, 36.5, 182, 36.5, C_GREEN, 1.2, '-');
    text(ax, 189, 41.8, {'e_{2,Y} /', 'e_{2,raw}'}, 'HorizontalAlignment', 'center', 'FontSize', 6.4, 'FontWeight', 'bold', 'Color', C_GREEN);

    draw_arrow(ax, 105, 36.5, 91, 36.5, C_GREEN, 1.2, '-');
    text(ax, 98, 40.5, 'e_{2,filt}', 'HorizontalAlignment', 'center', 'FontSize', 6.7, 'FontWeight', 'bold', 'Color', C_GREEN);

    % =====================================================================
    % 8. CÁC TUYẾN BUS LIÊN VÙNG TRÊN CÁC HÀNH LANG RIÊNG BIỆT (0 CHỒNG CHỮ)
    % =====================================================================

    % --- A) Robot (Vùng 5) -> TCRT5000 (Vùng 6A) ---
    draw_arrow(ax, 430, 104, 430, 61, C_BLACK, 1.3, '-');
    text(ax, 430, 82.0, {'(x_1, y_1, \phi)', 'quang học'}, 'HorizontalAlignment', 'center', ...
        'FontSize', 7.0, 'FontWeight', 'bold', 'Color', C_BLACK, 'BackgroundColor', 'w', 'Margin', 0.8);

    % --- B) Vùng 3 (SMC & Profiler) -> Vùng 4A (Drive Service Core 1) qua hành lang x = 206..218 ---
    % v_cmd từ 3B (200, 178) -> 4A (223, 178)
    draw_arrow(ax, 200, 178, 223, 178, C_BLUE, 1.35, '-');
    text(ax, 211.5, 182.2, 'v_{cmd}', 'HorizontalAlignment', 'center', 'FontSize', 7.3, 'FontWeight', 'bold', 'Color', C_BLUE);

    % Brake / S-curve từ 3A (116, 158) -> rãnh y=151.5 -> x=208 -> y=166 -> 4A (223, 166)
    draw_polyline_arrow(ax, [116 158; 116 151.5; 208 151.5; 208 166; 223 166], C_RED, 1.0, '--');
    text(ax, 211.5, 169.5, 'Brake/S-c', 'HorizontalAlignment', 'center', 'FontSize', 6.1, 'FontWeight', 'bold', 'Color', C_RED, 'BackgroundColor', 'w', 'Margin', 0.2);

    % w_cmd từ 3D (200, 122) -> x=212 -> y=154 -> 4A (223, 154)
    draw_polyline_arrow(ax, [200 122; 212 122; 212 154; 223 154], C_BLUE, 1.35, '-');
    text(ax, 212, 138.0, '\omega_{cmd}', 'HorizontalAlignment', 'center', ...
        'FontSize', 7.3, 'FontWeight', 'bold', 'Color', C_BLUE, 'BackgroundColor', 'w', 'Margin', 0.4);

    % --- C) HÀNH LANG TRÊN (y = 216..234): Vùng 1 & Vùng 2 -> Vùng 3 & Vùng 4 ---
    % C1: Luật tham số (2C) -> 2-DOF PI (A) & 2-DOF PI (B)
    plot(ax, [357.5 357.5], [240 223.5], '--', 'Color', C_ORANGE, 'LineWidth', 1.2);
    draw_polyline_arrow(ax, [357.5 223.5; 303 223.5; 303 207], C_ORANGE, 1.2, '--');
    text(ax, 330, 227.5, 'K_{p,A}, u_{dz,A}, e_{band}', 'HorizontalAlignment', 'center', ...
        'FontSize', 6.8, 'FontWeight', 'bold', 'Color', C_ORANGE, 'BackgroundColor', 'w', 'Margin', 0.3);

    % Nhánh xuống Kênh B đi theo khe x = 329.5 (giữa 2-DOF PI và DRV8871)
    draw_dot(ax, 329.5, 223.5, C_ORANGE);
    draw_polyline_arrow(ax, [329.5 223.5; 329.5 155; 303 155; 303 150], C_ORANGE, 1.15, '--');
    text(ax, 315, 157.5, 'K_{p,B}, u_{dz,B}', 'HorizontalAlignment', 'center', ...
        'FontSize', 6.4, 'FontWeight', 'bold', 'Color', C_ORANGE, 'BackgroundColor', 'w', 'Margin', 0.3);

    % C2: Phân loại tải m_g, load_class (2B) -> Mission FSM (3A) tại y = 228.5
    plot(ax, [300 300], [240 228.5], '-', 'Color', C_ORANGE, 'LineWidth', 1.15);
    draw_polyline_arrow(ax, [300 228.5; 62 228.5; 62 206], C_ORANGE, 1.15, '-');
    text(ax, 250, 231.0, 'm_g, load\_class \in \{1 kg (Trái), 2 kg (Phải)\}', ...
        'HorizontalAlignment', 'center', 'FontSize', 6.9, 'FontWeight', 'bold', 'Color', C_ORANGE, ...
        'BackgroundColor', 'w', 'Margin', 0.3);

    % Rẽ nhánh load_class xuống Gauss-Newton (6C) qua khe x = 215.5 -> y = 74.5
    draw_dot(ax, 215.5, 228.5, C_ORANGE);
    plot(ax, [215.5 215.5], [228.5 74.5], ':', 'Color', C_ORANGE, 'LineWidth', 1.15);
    draw_polyline_arrow(ax, [215.5 74.5; 230 74.5; 230 61], C_ORANGE, 1.15, ':');
    text(ax, 223, 77.2, '1/2 kg', 'HorizontalAlignment', 'center', 'FontSize', 6.3, 'FontWeight', 'bold', 'Color', C_ORANGE);

    % C3: Lệnh MATLAB/App (1B) -> 3A & Log 52B từ 3 -> 1C (đặt nhãn gọn tại y = 222)
    draw_arrow(ax, 112, 239, 112, 206, C_PURPLE, 1.15, '-');
    text(ax, 112, 221.8, 'mode, p_{SMC}, STOP', 'HorizontalAlignment', 'center', ...
        'FontSize', 6.5, 'FontWeight', 'bold', 'Color', C_PURPLE, 'BackgroundColor', 'w', 'Margin', 0.3);

    draw_arrow(ax, 177, 206, 177, 239, C_PURPLE, 1.15, '--');
    text(ax, 177, 221.8, 'Log 52B (100Hz)', 'HorizontalAlignment', 'center', ...
        'FontSize', 6.5, 'FontWeight', 'bold', 'Color', C_PURPLE, 'BackgroundColor', 'w', 'Margin', 0.3);

    % C4: NVS calib từ 1C -> 6B theo rãnh dọc x = 203.5 -> hành lang y = 74.5
    draw_polyline_arrow(ax, [201 247; 203.8 247; 203.8 74.5; 326 74.5; 326 61], C_PURPLE, 0.95, ':');
    text(ax, 294, 77.2, 'NVS: R_w[7], R_b[7] & Profile 5S/7S', 'HorizontalAlignment', 'center', ...
        'FontSize', 6.4, 'Color', C_PURPLE, 'BackgroundColor', 'w', 'Margin', 0.3);

    % --- D) HÀNH LANG DƯỚI (y = 70..94): 3 TUYẾN BUS ĐỘC LẬP, TUYỆT ĐỐI KHÔNG ĐÈ ZONE 6 ---

    % Tuyến D1 (y = 89.0): Bus Core 1 -> Core 0 (v_act, w_act, s_enc, v_ref/act, 200Hz Notify)
    plot(ax, [253 253], [99 89.0], '-', 'Color', C_TEAL, 'LineWidth', 1.25);
    plot(ax, [253 52],  [89.0 89.0], '-', 'Color', C_TEAL, 'LineWidth', 1.25);

    % Rẽ lên 3C (Adaptive Scheduler)
    draw_dot(ax, 52, 89.0, C_TEAL);
    draw_arrow(ax, 52, 89.0, 52, 99, C_TEAL, 1.15, '-');
    % Rẽ xuống 6E (Luenberger Observer)
    draw_arrow(ax, 52, 89.0, 52, 61, C_TEAL, 1.15, '-');
    % Rẽ xuống 6C (Gauss-Newton: s_enc mở cửa sổ ngã ba Y)
    draw_dot(ax, 246, 89.0, C_TEAL);
    draw_arrow(ax, 246, 89.0, 246, 61, C_TEAL, 1.05, '-');
    text(ax, 251.5, 82.0, 's_{enc}', 'HorizontalAlignment', 'left', 'FontSize', 6.6, 'FontWeight', 'bold', 'Color', C_TEAL);
    % Rẽ lên 3A/3B (s_enc, v_act) đi gọn theo rãnh bên phải 3D (x = 202.0 -> y=148.5 -> 3B & 3A)
    draw_dot(ax, 202.0, 89.0, C_TEAL);
    draw_polyline_arrow(ax, [202.0 89.0; 202.0 148.5; 150 148.5; 150 158], C_TEAL, 1.0, '-');
    draw_dot(ax, 150, 148.5, C_TEAL);
    draw_polyline_arrow(ax, [150 148.5; 104 148.5; 104 158], C_TEAL, 1.0, '-');

    % Tuyến D2 (y = 82.0): Hồi tiếp Ước lượng (6E: ê_3, ĉ) & Bộ lọc (6D: e_2,filt) -> Vùng 3D (SMC)
    draw_polyline_arrow(ax, [28 61; 28 82.0; 118 82.0; 118 99], C_GREEN, 1.25, '-');
    text(ax, 76, 84.3, 'ê_3 [rad], ĉ [1/m]', 'HorizontalAlignment', 'center', ...
        'FontSize', 6.8, 'FontWeight', 'bold', 'Color', C_GREEN, 'BackgroundColor', 'w', 'Margin', 0.3);

    draw_arrow(ax, 143, 61, 143, 99, C_GREEN, 1.25, '-');
    text(ax, 153, 82.0, 'e_{2,filt}', 'HorizontalAlignment', 'center', ...
        'FontSize', 6.9, 'FontWeight', 'bold', 'Color', C_GREEN, 'BackgroundColor', 'w', 'Margin', 0.3);

    % Đặt nhãn Bus Core 1 -> Core 0 SAU các đường dọc để nền trắng che gọn điểm cắt
    text(ax, 152, 91.2, 'Bus Core 1 \rightarrow Core 0: v_{act}, \omega_{act}, s_{enc}, v_{A/B,ref}, v_{A/B,đo}, Notify 200 Hz', ...
        'HorizontalAlignment', 'center', 'FontSize', 6.8, 'FontWeight', 'bold', 'Color', C_TEAL, ...
        'BackgroundColor', 'w', 'Margin', 0.35);

    % Tuyến D3 (y = 74.5 -> biên trái x = 9.2): e_2,filt, ΣI, n_act, line_ok -> 3A (Watchdog)
    draw_dot(ax, 143, 74.5, C_GREEN);
    draw_polyline_arrow(ax, [143 74.5; 9.2 74.5; 9.2 182; 14 182], C_GREEN, 1.05, '--');
    text(ax, 92, 76.6, 'e_{2,filt}, ΣI, n_{act}, line\_ok \rightarrow Watchdog', ...
        'HorizontalAlignment', 'center', 'FontSize', 6.4, 'FontWeight', 'bold', 'Color', C_GREEN, ...
        'BackgroundColor', 'w', 'Margin', 0.3);

    % =====================================================================
    % 9. CHÚ GIẢI PHÂN TẦNG TẦN SỐ & QUY ƯỚC NÉT VẼ (y = 3.0)
    % =====================================================================
    text(ax, 235, 3.0, ...
        'Quy ước: Nét liền = Luồng tín hiệu thời gian thực   |   Nét đứt = Lập lịch tham số / Xung kích / Ngắt an toàn   |   Nét chấm = NVS / Cửa sổ gating   |   Đa tần số: Core 1 (200 Hz) \leftrightarrow Core 0 (200 Hz cảm biến / 100 Hz Observer+GN / 50–160 ms SMC)', ...
        'HorizontalAlignment', 'center', 'FontSize', 7.5, 'FontAngle', 'italic', 'Color', C_GRAY);

    % =====================================================================
    % 10. XUẤT FILE SVG (VECTOR) & PNG (300 DPI)
    % =====================================================================
    if bw_mode
        base_name = 'Hinh_TongThe_ChuTrinhTinHieu_TrangDen';
    else
        base_name = 'Hinh_TongThe_ChuTrinhTinHieu_Mau';
    end

    svg_path = fullfile(out_dir, [base_name '.svg']);
    png_path = fullfile(out_dir, [base_name '.png']);

    export_fig_svg(fig, svg_path);
    export_fig_png(fig, png_path, 300);
    close(fig);

    fprintf('Đã xuất:\n  - %s\n  - %s\n', svg_path, png_path);
end


%% =========================================================================
%  CÁC HÀM PHỤ TRỢ VẼ VECTOR (ĐỒNG BỘ VỚI XuatHinhAnhBaoCao.m)
% =========================================================================

function draw_zone(ax, x, y, w, h, fcol, ecol, title_str)
    rx = min(0.05, 4.0 / w);
    ry = min(0.05, 4.0 / h);
    rectangle(ax, 'Position', [x, y, w, h], 'Curvature', [rx ry], ...
        'FaceColor', fcol, 'EdgeColor', ecol, 'LineWidth', 1.1, 'LineStyle', '--');
    text(ax, x + 3.2, y + h - 3.2, title_str, ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle', ...
        'FontSize', 7.7, 'FontWeight', 'bold', 'Color', ecol);
end

function draw_box(ax, x, y, w, h, fcol, ecol, lw, lines, fsz, tcol)
    rx = min(0.18, 2.8 / w);
    ry = min(0.18, 2.8 / h);
    rectangle(ax, 'Position', [x, y, w, h], 'Curvature', [rx ry], ...
        'FaceColor', fcol, 'EdgeColor', ecol, 'LineWidth', lw);
    text(ax, x + w/2, y + h/2, lines, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontSize', fsz, 'Color', tcol);
end

function draw_sum_x(ax, cx, cy, r, col)
    th = linspace(0, 2*pi, 60);
    patch(ax, cx + r*cos(th), cy + r*sin(th), 'w', 'EdgeColor', col, 'LineWidth', 1.15);
    d = r * cos(pi/4);
    plot(ax, [cx - d, cx + d], [cy - d, cy + d], '-', 'Color', col, 'LineWidth', 0.9);
    plot(ax, [cx - d, cx + d], [cy + d, cy - d], '-', 'Color', col, 'LineWidth', 0.9);
end

function draw_dot(ax, x, y, col)
    plot(ax, x, y, 'o', 'Color', col, 'MarkerFaceColor', col, 'MarkerSize', 3.6);
end

function draw_arrow(ax, x1, y1, x2, y2, col, lw, ls)
    dx = x2 - x1; dy = y2 - y1;
    L = hypot(dx, dy);
    if L < 1e-6, return; end
    ux = dx / L; uy = dy / L;
    px = -uy;    py = ux;
    hlen = min(2.6, 0.45 * L);
    hw   = 1.0;
    plot(ax, [x1, x2 - 0.72*hlen*ux], [y1, y2 - 0.72*hlen*uy], ...
        'LineStyle', ls, 'Color', col, 'LineWidth', lw);
    vx = [x2, x2 - hlen*ux + hw*px, x2 - hlen*ux - hw*px];
    vy = [y2, y2 - hlen*uy + hw*py, y2 - hlen*uy - hw*py];
    patch(ax, vx, vy, col, 'EdgeColor', 'none');
end

function draw_double_arrow(ax, x1, y1, x2, y2, col, lw, ls)
    dx = x2 - x1; dy = y2 - y1;
    L = hypot(dx, dy);
    if L < 1e-6, return; end
    ux = dx / L; uy = dy / L;
    px = -uy;    py = ux;
    hlen = min(2.4, 0.35 * L);
    hw   = 0.95;
    plot(ax, [x1 + 0.72*hlen*ux, x2 - 0.72*hlen*ux], ...
             [y1 + 0.72*hlen*uy, y2 - 0.72*hlen*uy], ...
             'LineStyle', ls, 'Color', col, 'LineWidth', lw);
    patch(ax, [x2, x2 - hlen*ux + hw*px, x2 - hlen*ux - hw*px], ...
              [y2, y2 - hlen*uy + hw*py, y2 - hlen*uy - hw*py], col, 'EdgeColor', 'none');
    patch(ax, [x1, x1 + hlen*ux + hw*px, x1 + hlen*ux - hw*px], ...
              [y1, y1 + hlen*uy + hw*py, y1 + hlen*uy - hw*py], col, 'EdgeColor', 'none');
end

function draw_polyline_arrow(ax, pts, col, lw, ls)
    n = size(pts, 1);
    if n < 2, return; end
    if n > 2
        plot(ax, pts(1:n-1, 1), pts(1:n-1, 2), 'LineStyle', ls, 'Color', col, 'LineWidth', lw);
    end
    draw_arrow(ax, pts(n-1, 1), pts(n-1, 2), pts(n, 1), pts(n, 2), col, lw, ls);
end

function export_fig_svg(fig, filepath)
    try
        exportgraphics(fig, filepath, 'ContentType', 'vector');
    catch
        pos = get(fig, 'Position');
        set(fig, 'Renderer', 'painters', ...
            'PaperUnits', 'centimeters', ...
            'PaperSize', pos(3:4), ...
            'PaperPosition', [0, 0, pos(3:4)]);
        print(fig, filepath, '-dsvg');
    end
end

function export_fig_png(fig, filepath, dpi)
    try
        exportgraphics(fig, filepath, 'Resolution', dpi);
    catch
        print(fig, filepath, '-dpng', sprintf('-r%d', dpi));
    end
end
