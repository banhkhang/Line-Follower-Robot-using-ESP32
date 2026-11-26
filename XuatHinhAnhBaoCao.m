% =========================================================================
%  export_report_figures.m
%  Xuất toàn bộ đồ thị dữ liệu và sơ đồ khối chuẩn học thuật (SVG)
%  Đề tài: Robot phân phối hàng hóa theo khối lượng (Mục 5.2 & Mục 6.1)
%
%  Yêu cầu: Đặt 9 file CSV cùng thư mục hiện hành với script này
%           (6 file gá đỡ/sàn gạch + 3 file mặt bạt sa bàn 0/1/2 kg).
%  Không yêu cầu Control System Toolbox hay Signal Processing Toolbox.
%  Đầu ra: Thư mục figures/ gồm 16 hình SVG có tên đầy đủ kèm caption:
%          - 3 hình sơ đồ nguyên lý & kiến trúc (Hình 5.1, 6.2, 6.6)
%          - 7 hình đồ thị dữ liệu chính (Hình 5.3 -> 6.5)
%          - 6 hình bổ sung (Hình 5.1b -> 6.6b)
% =========================================================================
clear; clc; close all;

% Đặt true nếu chỉ muốn xuất nhanh 3 hình sơ đồ mới (không cần 9 file CSV)
ONLY_SCHEMATICS = false;

%% 1. Cấu hình chung (Font Times New Roman, bảng màu tương phản màu & trắng-đen)
out_dir = fullfile(pwd, 'figures');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

set(0, 'DefaultAxesFontName',  'Times New Roman');
set(0, 'DefaultTextFontName',  'Times New Roman');
set(0, 'DefaultAxesFontSize',  11);
set(0, 'DefaultTextFontSize',  11);
set(0, 'DefaultLegendFontSize', 10.5);
set(0, 'DefaultFigureColor',   'w');

% Bảng màu tương phản cao (phân biệt rõ cả khi in màu lẫn in trắng-đen)
C_BLUE   = [0.00, 0.30, 0.65];   % Động cơ A / Đường chính
C_RED    = [0.80, 0.14, 0.12];   % Động cơ B / Đường đối chứng / Cảnh báo
C_GREEN  = [0.00, 0.50, 0.36];   % Đường phụ / Sau cải tiến / Mặt bạt sa bàn
C_ORANGE = [0.85, 0.44, 0.00];   % Tải +1 kg / Bậc 2 / Lập lịch tham số
C_GRAY   = [0.45, 0.45, 0.45];   % Vùng phụ / Đường gióng
C_BLACK  = [0.10, 0.10, 0.10];   % Tín hiệu đặt v_ref / Chuẩn

% Màu nền nhạt (solid, không dùng alpha để xuất vector SVG sạch tuyệt đối)
BG_BLUE   = [0.92, 0.96, 1.00];
BG_RED    = [1.00, 0.93, 0.92];
BG_GREEN  = [0.91, 0.97, 0.94];
BG_ORANGE = [1.00, 0.95, 0.89];
BG_GRAY   = [0.95, 0.95, 0.96];


%% =========================================================================
%  PHẦN A: 3 HÌNH SƠ ĐỒ NGUYÊN LÝ & KIẾN TRÚC (HÌNH 5.1, HÌNH 6.2, HÌNH 6.6)
% =========================================================================

%% -------------------------------------------------------------------------
%  HÌNH 5.1: Nguyên lý đo vận tốc kết hợp xung – thời gian M/T ở Ts = 0,005s
% --------------------------------------------------------------------------
fig = figure('Units', 'centimeters', 'Position', [2 2 16.5 9.6], 'Visible', 'off');
ax = axes(fig, 'Position', [0.01 0.01 0.98 0.98]); hold(ax, 'on');
xlim(ax, [0 165]); ylim(ax, [0 96]); daspect(ax, [1 1 1]); axis(ax, 'off');

% Tỉ lệ trục thời gian: 1 ms = 10 mm -> Ts = 5 ms = 50 mm
x_start = 32; x_end = 162;
x_k0 = 44; x_k1 = 94; x_k2 = 144;
Tp = 20.6; % Chu kỳ xung kênh A tại v = 0,3 m/s (~2,06 ms -> 4125 / 6188 us)
edges = [37.0, 57.6, 78.2, 98.8, 119.4, 140.0, 160.6];

% Nhãn hàng bên trái
text(ax, 2, 75.5, {'Nhịp lấy mẫu', '(200 Hz, T_s = 5 ms)'}, 'FontSize', 9.2, 'FontWeight', 'bold', 'Color', C_BLACK);
text(ax, 2, 54.5, {'Kênh A (IRAM ISR)', '(cạnh lên, N_{1x} = 330)'}, 'FontSize', 9.2, 'FontWeight', 'bold', 'Color', C_BLUE);
text(ax, 2, 34.5, {'Kênh B', '(lệch pha 90°)'}, 'FontSize', 9.2, 'FontWeight', 'bold', 'Color', C_GREEN);
text(ax, 2, 16.5, {'Cửa sổ đo M/T', '(timer 1 \mus)'}, 'FontSize', 9.2, 'FontWeight', 'bold', 'Color', C_BLACK);

% Các vạch gióng đứng tại nhịp lấy mẫu Ts = 5 ms
for xk = [x_k0, x_k1, x_k2]
    plot(ax, [xk xk], [14 87], '--', 'Color', C_GRAY, 'LineWidth', 0.9);
end

% Hàng 1: Trục nhịp lấy mẫu cố định Ts = 5 ms
draw_arrow(ax, x_start, 73, x_end + 1, 73, C_BLACK, 1.0, '-');
text(ax, x_end - 1, 70.5, 't', 'FontSize', 10, 'FontAngle', 'italic', 'FontWeight', 'bold');
tick_lbls = {'t_{k-1}', 't_k', 't_{k+1}'};
xk_vec = [x_k0, x_k1, x_k2];
for i = 1:3
    draw_arrow(ax, xk_vec(i), 73, xk_vec(i), 78.5, C_BLACK, 1.2, '-');
    text(ax, xk_vec(i), 80.5, tick_lbls{i}, 'HorizontalAlignment', 'center', ...
        'FontSize', 9.5, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', 0.5);
end

% So sánh cửa sổ cố định Ts = 5 ms ở phía trên
draw_double_arrow(ax, x_k0, 86, x_k1, 86, C_ORANGE, 1.2, '-');
text(ax, (x_k0 + x_k1)/2, 91.2, 'Chu kỳ cố định T_s = 5 ms (lệch pha với xung \rightarrow sai số \pm1 xung ở Method M)', ...
    'HorizontalAlignment', 'center', 'FontSize', 9.2, 'FontWeight', 'bold', 'Color', C_ORANGE);

% Hàng 2: Dạng sóng vuông Kênh A (mức thấp y=49, mức cao y=60)
[xa, ya] = build_square_wave(x_start, x_end, edges, Tp/2, 49, 60);
plot(ax, xa, ya, '-', 'Color', C_BLUE, 'LineWidth', 1.45);
for i = 1:numel(edges)
    if edges(i) <= x_end - 1
        % Mũi tên đánh dấu cạnh lên kênh A
        draw_arrow(ax, edges(i), 49.5, edges(i), 56.5, C_BLUE, 1.1, '-');
    end
end

% Chèn 1 xung nhiễu EMI hẹp (< 80 us) trong đoạn mức thấp giữa e3 và e4
x_emi = 112.5;
plot(ax, [x_emi x_emi x_emi+0.8 x_emi+0.8], [49 60 60 49], '-', 'Color', C_RED, 'LineWidth', 1.3);
plot(ax, x_emi + 0.4, 61.5, 'x', 'Color', C_RED, 'MarkerSize', 6, 'LineWidth', 1.4);
plot(ax, [x_emi + 0.8, 117], [61.5, 65.5], '-', 'Color', C_RED, 'LineWidth', 0.8);
text(ax, 118, 65.8, 'Nhiễu EMI (< 80 \mus) bị loại', 'FontSize', 8.8, ...
    'FontWeight', 'bold', 'Color', C_RED, 'BackgroundColor', 'w', 'Margin', 0.5);

% Hàng 3: Dạng sóng vuông Kênh B lệch pha 90 độ (mức thấp y=30, mức cao y=39)
edges_b = edges - Tp/4;
[xb, yb] = build_square_wave(x_start, x_end, edges_b, Tp/2, 30, 39);
plot(ax, xb, yb, '-', 'Color', C_GREEN, 'LineWidth', 1.35);
for i = 1:numel(edges)
    if edges(i) <= x_end - 1
        plot(ax, [edges(i) edges(i)], [39 49], ':', 'Color', C_GREEN, 'LineWidth', 0.8);
        plot(ax, edges(i), 39, 'o', 'Color', C_GREEN, 'MarkerFaceColor', C_GREEN, 'MarkerSize', 4.5);
    end
end
plot(ax, [119.4, 123], [39.5, 43.8], '-', 'Color', C_GREEN, 'LineWidth', 0.8);
text(ax, 100, 46.0, 'Đọc GPIO\_IN\_REG = 1 \rightarrow chiều thuận', ...
    'FontSize', 8.8, 'FontWeight', 'bold', 'Color', C_GREEN, 'BackgroundColor', 'w', 'Margin', 0.5);

% Đường gióng từ 3 mốc cạnh lên cuối cùng xuống băng đo M/T
key_e = [edges(1), edges(3), edges(6)]; % t_last[k-1], t_last[k], t_last[k+1]
key_lbl = {'t_{last}[k-1]', 't_{last}[k]', 't_{last}[k+1]'};
for i = 1:3
    plot(ax, [key_e(i) key_e(i)], [15 49], ':', 'Color', C_BLUE, 'LineWidth', 1.1);
    plot(ax, key_e(i), 49, 'o', 'Color', C_BLUE, 'MarkerFaceColor', C_BLUE, 'MarkerSize', 4.5);
    text(ax, key_e(i), 22.8, key_lbl{i}, 'HorizontalAlignment', 'center', ...
        'FontSize', 8.8, 'Color', C_BLUE, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'Margin', 0.4);
end

% Hàng 4: Hai khoảng thời gian thực M/T ứng với v = 0,3 m/s (4125 us & 6188 us)
draw_double_arrow(ax, key_e(1), 17.5, key_e(2), 17.5, C_BLUE, 1.5, '-');
text(ax, (key_e(1)+key_e(2))/2, 12.2, '\DeltaN = 2,  \Deltat_{\mus} \approx 4125 \mus', ...
    'HorizontalAlignment', 'center', 'FontSize', 9.2, 'FontWeight', 'bold', 'Color', C_BLUE);

draw_double_arrow(ax, key_e(2), 17.5, key_e(3), 17.5, C_GREEN, 1.5, '-');
text(ax, (key_e(2)+key_e(3))/2, 12.2, '\DeltaN = 3,  \Deltat_{\mus} \approx 6188 \mus', ...
    'HorizontalAlignment', 'center', 'FontSize', 9.2, 'FontWeight', 'bold', 'Color', C_GREEN);

% Dòng chốt ý nghĩa
text(ax, 97, 4.2, 'Đo đúng \Deltat_{\mus} giữa hai cạnh lên cuối (thay vì chia cho T_s cố định) \rightarrow sai số lượng tử hóa giảm từ 10,3% xuống 0,024%', ...
    'HorizontalAlignment', 'center', 'FontSize', 9.0, 'FontAngle', 'italic', 'Color', C_BLACK);

fname_5_1 = 'Hình 5.1. Nguyên lý đo vận tốc M-T ở chu kỳ Ts = 0,005s.svg';
export_fig_svg(fig, fullfile(out_dir, fname_5_1));
close(fig); fprintf('Đã xuất figures/%s\n', fname_5_1);

%% -------------------------------------------------------------------------
%  HÌNH 6.2: Sơ đồ khối bộ điều khiển vận tốc 2-DOF PI rời rạc (Trắng – Đen)
% --------------------------------------------------------------------------
fig = figure('Units', 'centimeters', 'Position', [2 2 18.2 9.8], 'Visible', 'off');
ax = axes(fig, 'Position', [0.01 0.01 0.98 0.98]); hold(ax, 'on');
xlim(ax, [0 182]); ylim(ax, [0 98]); daspect(ax, [1 1 1]); axis(ax, 'off');

% Khối 1: Feedfoward + bù vùng chết
draw_box(ax, 24, 68, 57, 14, 'w', C_BLACK, 1.2, { ...
    '\bfFeedfoward + Bù vùng chết'}, 8.6, C_BLACK);

% Bộ cộng S1 (sai số) và S2 (tổng lệnh)
draw_sum(ax, 23, 52, 3.0, C_BLACK);
text(ax, 18.8, 54.6, '+', 'FontSize', 10, 'FontWeight', 'bold', 'Color', C_BLACK);
text(ax, 25.0, 47.4, '–', 'FontSize', 11, 'FontWeight', 'bold', 'Color', C_BLACK);

draw_sum(ax, 90, 52, 3.0, C_BLACK);
text(ax, 87.3, 56.2, '+', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C_BLACK);
text(ax, 85.3, 54.5, '+', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C_BLACK);
text(ax, 87.3, 47.3, '+', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C_BLACK);

% Khối 2: Tỉ lệ P
draw_box(ax, 46, 46, 22, 12, 'w', C_BLACK, 1.2, { ...
    '\bfKhâu tỉ lệ'}, 8.6, C_BLACK);

% Khối 3 & 4: Cổng điều kiện và Tích phân Tustin
draw_box(ax, 36, 23, 20, 13, 'w', C_BLACK, 1.2, { ...
    '\bfĐiều kiện', '|e[k]| < e_{band}'}, 8.4, C_BLACK);
draw_box(ax, 60, 23, 24, 13, 'w', C_BLACK, 1.2, { ...
    '\bfTích phân Tustin'}, 8.4, C_BLACK);

% Khối 5: Bão hòa bất đối xứng (x = 104..124, y = 44..60 -> trung điểm x = 114, y = 52)
draw_box(ax, 104, 44, 20, 16, 'w', C_BLACK, 1.25, { ...
    '\bfBão hòa', '\bfbất đối xứng'}, 8.4, C_BLACK);

% Khối 6 & 7: DRV8871 và Động cơ (đẩy sang phải để thoáng khoảng u[k])
draw_box(ax, 134, 44, 17, 16, 'w', C_BLACK, 1.2, { ...
    '\bfDRV8871'}, 8.5, C_BLACK);
draw_box(ax, 156, 44, 16, 16, 'w', C_BLACK, 1.2, { ...
    '\bfĐộng cơ'}, 8.5, C_BLACK);

% Khối 8: Đo vận tốc Encoder
draw_box(ax, 120, 1.5, 38, 7.5, 'w', C_BLACK, 1.15, { ...
    '\bfVận tốc từ Encoder'}, 8.4, C_BLACK);

% Các đường tín hiệu chính
draw_arrow(ax, 2, 52, 20, 52, C_BLACK, 1.2, '-');
text(ax, 6.5, 55.2, 'v_{ref}[k]', 'HorizontalAlignment', 'center', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C_BLACK);
draw_dot(ax, 12, 52, C_BLACK);

% Nhánh lên Feedforward
draw_polyline_arrow(ax, [12 52; 12 75; 24 75], C_BLACK, 1.15, '-');
draw_polyline_arrow(ax, [81 75; 90 75; 90 55], C_BLACK, 1.15, '-');
text(ax, 85.5, 77.6, 'u_{ff}[k]', 'HorizontalAlignment', 'center', 'FontSize', 8.8, 'Color', C_BLACK);

% Đường khóa biên dưới động Anti-Plugging -> vào đúng trung điểm cạnh trên (114, 60)
draw_dot(ax, 12, 75, C_BLACK);
draw_polyline_arrow(ax, [12 75; 12 91.5; 114 91.5; 114 60], C_BLACK, 1.05, '--');
text(ax, 63, 94.3, 'Khóa biên dưới (Short Brake / Anti-Plugging) khi v_{ref} > 0,05 m/s', ...
    'HorizontalAlignment', 'center', 'FontSize', 8.6, 'FontWeight', 'bold', 'Color', C_BLACK);

% Nhánh sai số e[k] vào P và I
draw_arrow(ax, 26, 52, 46, 52, C_BLACK, 1.15, '-');
text(ax, 33, 55.0, 'e[k]', 'HorizontalAlignment', 'center', 'FontSize', 9.0, 'FontWeight', 'bold', 'Color', C_BLACK);
draw_dot(ax, 32, 52, C_BLACK);
draw_polyline_arrow(ax, [32 52; 32 29.5; 36 29.5], C_BLACK, 1.15, '-');
draw_arrow(ax, 56, 29.5, 60, 29.5, C_BLACK, 1.15, '-');

% Đầu ra P và I vào S2
draw_arrow(ax, 68, 52, 87, 52, C_BLACK, 1.15, '-');
text(ax, 77.5, 54.6, 'u_P[k]', 'HorizontalAlignment', 'center', 'FontSize', 8.6, 'Color', C_BLACK);
draw_polyline_arrow(ax, [84 29.5; 90 29.5; 90 49], C_BLACK, 1.15, '-');
text(ax, 93.8, 36.5, 'I[k]', 'FontSize', 8.6, 'Color', C_BLACK);

% Đường Anti-windup -> đi từ đúng trung điểm cạnh dưới (114, 44) về đáy khối Tích phân
draw_polyline_arrow(ax, [114 44; 114 15.5; 72 15.5; 72 23], C_BLACK, 1.05, '--');
text(ax, 93, 13.0, 'Anti-windup', ...
    'HorizontalAlignment', 'center', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', C_BLACK);

% Từ S2 -> Bão hòa (kéo dài 11 mm) -> DRV8871 (dài 10 mm) -> Động cơ -> Ngõ ra v[k]
draw_arrow(ax, 93, 52, 104, 52, C_BLACK, 1.15, '-');
text(ax, 98.5, 55.2, 'u_{unsat}', 'HorizontalAlignment', 'center', 'FontSize', 8.5, 'Color', C_BLACK);
draw_arrow(ax, 124, 52, 134, 52, C_BLACK, 1.15, '-');
text(ax, 129.0, 55.2, 'u[k]', 'HorizontalAlignment', 'center', 'FontSize', 8.8, 'FontWeight', 'bold', 'Color', C_BLACK);
draw_arrow(ax, 151, 52, 156, 52, C_BLACK, 1.15, '-');
draw_arrow(ax, 172, 52, 180, 52, C_BLACK, 1.2, '-');
text(ax, 176.5, 55.5, 'v[k]', 'HorizontalAlignment', 'center', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', C_BLACK);

% Đường hồi tiếp vận tốc qua Encoder
draw_dot(ax, 176, 52, C_BLACK);
draw_polyline_arrow(ax, [176 52; 176 5.25; 158 5.25], C_BLACK, 1.15, '-');
draw_polyline_arrow(ax, [120 5.25; 23 5.25; 23 49], C_BLACK, 1.15, '-');
text(ax, 70, 7.6, 'Vận tốc thực v[k] (m/s)', 'HorizontalAlignment', 'center', ...
    'FontSize', 8.8, 'FontWeight', 'bold', 'Color', C_BLACK);

fname_6_2 = 'Hình 6.2. Sơ đồ khối bộ điều khiển vận tốc 2-DOF PI rời rạc.svg';
export_fig_svg(fig, fullfile(out_dir, fname_6_2));
close(fig); fprintf('Đã xuất figures/%s\n', fname_6_2);

%% -------------------------------------------------------------------------
%  HÌNH 6.6: Kiến trúc lập lịch tham số kết nối Loadcell HX711 – Vòng trong – Vòng ngoài (Trắng – Đen)
% --------------------------------------------------------------------------
fig = figure('Units', 'centimeters', 'Position', [1.5 1.5 19.6 12.0], 'Visible', 'off');
ax = axes(fig, 'Position', [0.01 0.01 0.98 0.98]); hold(ax, 'on');
xlim(ax, [0 226]); ylim(ax, [0 138]); daspect(ax, [1 1 1]); axis(ax, 'off');

W = 22;   % Chiều rộng thống nhất cho TẤT CẢ các khối
H = 11;   % Chiều cao tiêu chuẩn các khối
FS_BOX = 7.8;
FS_SIG = 8.2;

% =========================================================================
% TẦNG 1: m_load -> LOADCELL HX711 -> TÍNH TOÁN KHỐI LƯỢNG -> BỘ LUẬT THAM SỐ
% =========================================================================
draw_box(ax, 24,  107, W, H, 'w', C_BLACK, 1.15, {'\bfLoadcell', '\bfHX711'}, FS_BOX, C_BLACK);
draw_box(ax, 62,  107, W, H, 'w', C_BLACK, 1.15, {'\bfTính toán', '\bfkhối lượng'}, FS_BOX, C_BLACK);
draw_box(ax, 100, 107, W, H, 'w', C_BLACK, 1.15, {'\bfBộ luật', '\bftham số'}, FS_BOX, C_BLACK);

% m_load đi vào node (35, 129) bằng đường thẳng (không mũi tên), từ node mới rẽ mũi tên
plot(ax, [10 35], [129 129], '-', 'Color', C_BLACK, 'LineWidth', 1.15);
text(ax, 22.5, 132.2, 'm_{load}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_dot(ax, 35, 129, C_BLACK);
draw_arrow(ax, 35, 129, 35, 118, C_BLACK, 1.15, '-');
draw_polyline_arrow(ax, [35 129; 203 129; 203 92], C_BLACK, 1.15, '-');

% Tín hiệu điện V_load -> khối lượng đo m_{load,đo}
draw_arrow(ax, 46, 112.5, 62, 112.5, C_BLACK, 1.1, '-');
text(ax, 54.0, 115.6, 'V_{load}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 84, 112.5, 100, 112.5, C_BLACK, 1.1, '-');
text(ax, 92.0, 115.6, 'm_{load,đo}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

% =========================================================================
% ĐƯỜNG LẬP LỊCH THAM SỐ XUYÊN CHÉO QUA 2 KHỐI 2-DOF PI (VẼ TRƯỚC HỘP ĐỂ KHÔNG ĐÈ CHỮ)
% =========================================================================
% Nhánh A: rẽ phải ở node -> đi xuống -> chéo từ phải sang trái xuyên qua PI (A)
plot(ax, [111 111], [107 98], '--', 'Color', C_BLACK, 'LineWidth', 1.05);
draw_dot(ax, 111, 98, C_BLACK);
draw_polyline_arrow(ax, [111 98; 124 98; 124 93; 98 76], C_BLACK, 1.05, '--');
text(ax, 126.0, 96.5, 'K_{p,A}, u_{dz,A}, e_{band}', 'HorizontalAlignment', 'left', ...
    'FontSize', 7.6, 'Color', C_BLACK);

% Nhánh B: rẽ trái qua khe x = 85.5 (tránh hoàn toàn e_A) -> xuyên chéo qua PI (B)
draw_polyline_arrow(ax, [111 98; 85.5 98; 85.5 55; 99 55; 124 36.5], C_BLACK, 1.05, '--');
text(ax, 101.0, 57.5, 'K_{p,B}, u_{dz,B}, e_{band}', 'HorizontalAlignment', 'left', ...
    'FontSize', 7.6, 'Color', C_BLACK);

% =========================================================================
% VÒNG NGOÀI: BỘ TỔNG -> BỘ ĐIỀU KHIỂN BÁM LINE -> ĐỘNG HỌC VI SAI
% =========================================================================
draw_sum_x(ax, 15, 65, 2.8, C_BLACK);
draw_arrow(ax, 2, 65, 12.2, 65, C_BLACK, 1.1, '-');
text(ax, 5.5, 68.2, 'e_{ref}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
text(ax, 10.6, 67.2, '+', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', C_BLACK);
text(ax, 17.2, 60.8, '–', 'FontSize', 10.0, 'FontWeight', 'bold', 'Color', C_BLACK);

draw_box(ax, 24, 59.5, W, H, 'w', C_BLACK, 1.15, {'\bfBộ điều khiển', '\bfbám line'}, FS_BOX, C_BLACK);
draw_box(ax, 55, 59.5, W, H, 'w', C_BLACK, 1.15, {'\bfĐộng học vi sai', '\bf+ S-curve'}, FS_BOX, C_BLACK);

draw_arrow(ax, 17.8, 65, 24, 65, C_BLACK, 1.1, '-');
text(ax, 20.9, 67.8, 'e', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 46, 65, 55, 65, C_BLACK, 1.1, '-');
text(ax, 50.5, 67.8, 'v, \omega', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

% =========================================================================
% VÒNG TRONG – KÊNH A (TRÊN)
% =========================================================================
draw_sum_x(ax, 91, 84.5, 2.8, C_BLACK);
text(ax, 86.8, 86.8, '+', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', C_BLACK);
text(ax, 93.2, 80.3, '–', 'FontSize', 10.0, 'FontWeight', 'bold', 'Color', C_BLACK);

draw_box(ax, 100, 79, W, H, 'w', C_BLACK, 1.15, {'\bf2-DOF PI (A)', '+ Short Brake'}, FS_BOX, C_BLACK);
draw_box(ax, 130, 79, W, H, 'w', C_BLACK, 1.15, {'\bfDRV8871 (A)'}, FS_BOX, C_BLACK);
draw_box(ax, 160, 79, W, H, 'w', C_BLACK, 1.15, {'\bfĐộng cơ A'}, FS_BOX, C_BLACK);

% Tín hiệu thuận Kênh A
draw_polyline_arrow(ax, [77 68; 81 68; 81 84.5; 88.2 84.5], C_BLACK, 1.1, '-');
text(ax, 80.0, 87.8, 'v_{ref,A}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 93.8, 84.5, 100, 84.5, C_BLACK, 1.1, '-');
text(ax, 96.9, 87.3, 'e_A', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 122, 84.5, 130, 84.5, C_BLACK, 1.1, '-');
text(ax, 126.0, 87.3, 'u_A', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 152, 84.5, 160, 84.5, C_BLACK, 1.1, '-');
text(ax, 156.0, 87.3, 'V_A', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 182, 84.5, 192, 84.5, C_BLACK, 1.1, '-');
text(ax, 187.0, 87.3, 'v_A', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

% Hồi tiếp Kênh A: Encoder A -> Tính toán vận tốc (nới sang trái để thoáng N_A) -> S_A
draw_box(ax, 160, 62, W, H, 'w', C_BLACK, 1.1, {'\bfEncoder A'}, FS_BOX, C_BLACK);
draw_box(ax, 124, 62, W, H, 'w', C_BLACK, 1.1, {'\bfTính toán', '\bfvận tốc'}, FS_BOX, C_BLACK);
draw_dot(ax, 187, 84.5, C_BLACK);
draw_polyline_arrow(ax, [187 84.5; 187 67.5; 182 67.5], C_BLACK, 1.05, '-');
draw_arrow(ax, 160, 67.5, 146, 67.5, C_BLACK, 1.05, '-');
text(ax, 153.0, 70.4, 'N_A', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_polyline_arrow(ax, [124 67.5; 91 67.5; 91 81.7], C_BLACK, 1.05, '-');
text(ax, 108.0, 70.4, 'v_{A,đo}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

% =========================================================================
% VÒNG TRONG – KÊNH B (DƯỚI)
% =========================================================================
draw_sum_x(ax, 91, 45.5, 2.8, C_BLACK);
text(ax, 86.8, 47.8, '+', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', C_BLACK);
text(ax, 93.2, 41.3, '–', 'FontSize', 10.0, 'FontWeight', 'bold', 'Color', C_BLACK);

draw_box(ax, 100, 40, W, H, 'w', C_BLACK, 1.15, {'\bf2-DOF PI (B)', '+ Short Brake'}, FS_BOX, C_BLACK);
draw_box(ax, 130, 40, W, H, 'w', C_BLACK, 1.15, {'\bfDRV8871 (B)'}, FS_BOX, C_BLACK);
draw_box(ax, 160, 40, W, H, 'w', C_BLACK, 1.15, {'\bfĐộng cơ B'}, FS_BOX, C_BLACK);

% Tín hiệu thuận Kênh B
draw_polyline_arrow(ax, [77 62; 81 62; 81 45.5; 88.2 45.5], C_BLACK, 1.1, '-');
text(ax, 80.0, 42.0, 'v_{ref,B}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 93.8, 45.5, 100, 45.5, C_BLACK, 1.1, '-');
text(ax, 96.9, 48.3, 'e_B', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 122, 45.5, 130, 45.5, C_BLACK, 1.1, '-');
text(ax, 126.0, 48.3, 'u_B', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 152, 45.5, 160, 45.5, C_BLACK, 1.1, '-');
text(ax, 156.0, 48.3, 'V_B', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_arrow(ax, 182, 45.5, 192, 45.5, C_BLACK, 1.1, '-');
text(ax, 187.0, 48.3, 'v_B', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

% Hồi tiếp Kênh B: Encoder B -> Tính toán vận tốc (nới sang trái để thoáng N_B) -> S_B
draw_box(ax, 160, 23, W, H, 'w', C_BLACK, 1.1, {'\bfEncoder B'}, FS_BOX, C_BLACK);
draw_box(ax, 124, 23, W, H, 'w', C_BLACK, 1.1, {'\bfTính toán', '\bfvận tốc'}, FS_BOX, C_BLACK);
draw_dot(ax, 187, 45.5, C_BLACK);
draw_polyline_arrow(ax, [187 45.5; 187 28.5; 182 28.5], C_BLACK, 1.05, '-');
draw_arrow(ax, 160, 28.5, 146, 28.5, C_BLACK, 1.05, '-');
text(ax, 153.0, 31.4, 'N_B', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_polyline_arrow(ax, [124 28.5; 91 28.5; 91 42.7], C_BLACK, 1.05, '-');
text(ax, 108.0, 31.4, 'v_{B,đo}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

% =========================================================================
% BÊN PHẢI & HỒI TIẾP VÒNG NGOÀI: ROBOT -> CẢM BIẾN DÒ LINE -> TÍNH TOÁN SAI SỐ
% =========================================================================
draw_box(ax, 192, 38, W, 54, 'w', C_BLACK, 1.2, {'\bfRobot vi sai', '\bftrên mặt bạt'}, FS_BOX, C_BLACK);
draw_arrow(ax, 214, 65, 224, 65, C_BLACK, 1.15, '-');
text(ax, 219.5, 68.5, 'x, y, \theta', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

draw_box(ax, 136, 4, W, H, 'w', C_BLACK, 1.15, {'\bfCảm biến', '\bfdò line'}, FS_BOX, C_BLACK);
draw_box(ax, 96,  4, W, H, 'w', C_BLACK, 1.15, {'\bfTính toán', '\bfsai số'}, FS_BOX, C_BLACK);

draw_dot(ax, 219, 65, C_BLACK);
draw_polyline_arrow(ax, [219 65; 219 9.5; 158 9.5], C_BLACK, 1.1, '-');
draw_arrow(ax, 136, 9.5, 118, 9.5, C_BLACK, 1.1, '-');
text(ax, 127.0, 12.6, 'V_{1, 2, ..., 7}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);
draw_polyline_arrow(ax, [96 9.5; 15 9.5; 15 62.2], C_BLACK, 1.1, '-');
text(ax, 55.5, 12.6, 'e_{line}', 'HorizontalAlignment', 'center', 'FontSize', FS_SIG, 'Color', C_BLACK);

fname_6_6 = 'Hình 6.6. Kiến trúc lập lịch tham số kết nối Loadcell HX711 với vòng trong và vòng ngoài.svg';
export_fig_svg(fig, fullfile(out_dir, fname_6_6));
close(fig); fprintf('Đã xuất figures/%s\n', fname_6_6);


%% =========================================================================
%  2. NẠP DỮ LIỆU THỰC NGHIỆM TỪ 9 FILE CSV
% =========================================================================
Ts      = 0.005;                  % Chu kỳ lấy mẫu [s] (200 Hz)
fs      = 1 / Ts;                 % Tần số lấy mẫu [Hz]
D_wheel = 0.065;                  % Đường kính bánh xe [m]
C_wheel = pi * D_wheel;           % Chu vi bánh xe = 0.20420 m
RPM2MPS = C_wheel / 60.0;         % Hệ số quy đổi RPM -> m/s
N_1x    = 330;                    % Xung/vòng chế độ M/T 1x
N_4x    = 1320;                   % Xung/vòng chế độ 4x

f_id_A    = 'sysid_motorA_id30_20261121_225438.csv';
f_id_B    = 'sysid_motorB_id30_20261121_225604.csv';
f_cl_4x   = 'cl10_bench_AB_20261121_232511.csv';
f_cl_1x   = 'cl10_bench_AB_20261121_234337.csv';
f_fl_0kg  = 'cl10_floor_13kg_AB_20261122_225857.csv';
f_fl_1kg  = 'cl10_floor_1kg_AB_20261122_230039.csv';
f_mat_0kg = 'cl10_floor_13kg_AB_20261123_130312.csv';
f_mat_1kg = 'cl10_floor_1kg_AB_20261123_130443.csv';
f_mat_2kg = 'cl10_floor_2kg_AB_20261123_130547.csv';

req_files = {f_id_A, f_id_B, f_cl_4x, f_cl_1x, f_fl_0kg, f_fl_1kg, ...
             f_mat_0kg, f_mat_1kg, f_mat_2kg};
for k = 1:numel(req_files)
    if ~isfile(req_files{k})
        error('Không tìm thấy file CSV: %s trong thư mục hiện hành! (Hoặc đặt ONLY_SCHEMATICS = true để chỉ vẽ 3 sơ đồ)', req_files{k});
    end
end

fprintf('Đang nạp dữ liệu từ 9 file CSV...\n');
T_idA  = readtable(f_id_A);
T_idB  = readtable(f_id_B);
T_cl4x = readtable(f_cl_4x);
T_cl1x = readtable(f_cl_1x);
T_fl0  = readtable(f_fl_0kg);
T_fl1  = readtable(f_fl_1kg);
T_mat0 = readtable(f_mat_0kg);
T_mat1 = readtable(f_mat_1kg);
T_mat2 = readtable(f_mat_2kg);

t_id  = double(T_idA.seq) * Ts;
u_id  = double(T_idA.pwm_pct);
rpmA  = double(T_idA.rpm_a);
rpmB  = double(T_idB.rpm_b);
if mean(rpmA) < 0, rpmA = -rpmA; end
if mean(rpmB) < 0, rpmB = -rpmB; end
vA_id = rpmA * RPM2MPS;
vB_id = rpmB * RPM2MPS;

Kdc_A_rpm = 3.453;   Kdc_A = 0.01175;  tau_A = 0.0762;  udz_A = 2.40;  tau2_A = 1/257.77;
Kdc_B_rpm = 3.601;   Kdc_B = 0.01225;  tau_B = 0.0952;  udz_B = 1.68;  tau2_B = 1/318.50;

vA_m1 = sim_first_order_zoh(u_id, Kdc_A, tau_A, udz_A, Ts);
vB_m1 = sim_first_order_zoh(u_id, Kdc_B, tau_B, udz_B, Ts);
vA_m2 = sim_second_order_zoh(u_id, Kdc_A, tau_A, tau2_A, udz_A, Ts);
vB_m2 = sim_second_order_zoh(u_id, Kdc_B, tau_B, tau2_B, udz_B, Ts);

m_est = (t_id >= 1.0)  & (t_id <= 20.0);
m_val = (t_id >= 20.0) & (t_id <= 29.0);
fitA_est = calc_nrmse_fit(vA_id(m_est), vA_m1(m_est));
fitA_val = calc_nrmse_fit(vA_id(m_val), vA_m1(m_val));
fitB_est = calc_nrmse_fit(vB_id(m_est), vB_m1(m_est));
fitB_val = calc_nrmse_fit(vB_id(m_val), vB_m1(m_val));
fprintf('Kiểm chứng Fit G1(s): A = %.2f%% / %.2f%% | B = %.2f%% / %.2f%%\n', ...
    fitA_est, fitA_val, fitB_est, fitB_val);


%% =========================================================================
%  PHẦN B: 7 HÌNH ĐỒ THỊ DỮ LIỆU CHÍNH TRONG BÁO CÁO (HÌNH 5.3 -> HÌNH 6.5)
% =========================================================================

%% -------------------------------------------------------------------------
%  HÌNH 5.3: Tín hiệu kích thích PWM và đáp ứng tốc độ 30s trên gá đỡ
% --------------------------------------------------------------------------
fig = figure('Units', 'centimeters', 'Position', [2 2 16 10.5], 'Visible', 'off');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile; hold(ax1, 'on');
patch(ax1, [1 20 20 1],   [0 0 95 95], [0.90 0.95 1.00], 'EdgeColor', 'none', 'FaceAlpha', 0.45);
patch(ax1, [20 29 29 20], [0 0 95 95], [1.00 0.93 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.55);
stairs(ax1, t_id, u_id, 'Color', C_BLACK, 'LineWidth', 1.4);
xline(ax1, 20, '--', 'Color', C_GRAY, 'LineWidth', 1.1);
text(ax1, 10.5, 90, 'Tập ước lượng Estimation (1 – 20s)', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'FontSize', 10.5, 'Color', C_BLUE);
text(ax1, 24.5, 90, 'Tập kiểm chứng Validation (20 – 29s)', 'HorizontalAlignment', 'center', ...
    'FontWeight', 'bold', 'FontSize', 10.5, 'Color', C_RED);
xlim(ax1, [0 30]); ylim(ax1, [0 98]);
ylabel(ax1, 'Độ rộng xung PWM (%)', 'FontSize', 11.5);
style_axes(ax1);

ax2 = nexttile; hold(ax2, 'on');
patch(ax2, [1 20 20 1],   [0 0 340 340], [0.90 0.95 1.00], 'EdgeColor', 'none', 'FaceAlpha', 0.45);
patch(ax2, [20 29 29 20], [0 0 340 340], [1.00 0.93 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.55);
pA = plot(ax2, t_id, rpmA, '-',  'Color', C_BLUE, 'LineWidth', 1.35);
pB = plot(ax2, t_id, rpmB, '--', 'Color', C_RED,  'LineWidth', 1.35);
xline(ax2, 20, '--', 'Color', C_GRAY, 'LineWidth', 1.1);
xlim(ax2, [0 30]); ylim(ax2, [0 335]);
xlabel(ax2, 'Thời gian (s)', 'FontSize', 11.5);
ylabel(ax2, 'Tốc độ quay trục ra (RPM)', 'FontSize', 11.5);
legend(ax2, [pA pB], {'Động cơ A (trái)', 'Động cơ B (phải)'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_axes(ax2);
linkaxes([ax1 ax2], 'x');

fname = 'Hình 5.3. Tín hiệu PWM nhiều mức và đáp ứng tốc độ 30s trên gá đỡ.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 5.4: Đặc tính tĩnh tốc độ xác lập theo PWM & vùng chết u_dz
% --------------------------------------------------------------------------
idx_chg = [1; find(diff(u_id) ~= 0) + 1; numel(u_id) + 1];
ss_u = []; ss_rpmA = []; ss_rpmB = [];
for i = 1:(numel(idx_chg) - 1)
    s_i = idx_chg(i); e_i = idx_chg(i+1) - 1;
    if u_id(s_i) >= 20
        t_end = t_id(e_i);
        m_ss = (t_id >= t_end - 0.80) & (t_id <= t_end - 0.02);
        ss_u(end+1, 1)    = u_id(s_i); %#ok<SAGROW>
        ss_rpmA(end+1, 1) = mean(rpmA(m_ss)); %#ok<SAGROW>
        ss_rpmB(end+1, 1) = mean(rpmB(m_ss)); %#ok<SAGROW>
    end
end
[ss_u, ord] = sort(ss_u);
ss_rpmA = ss_rpmA(ord); ss_rpmB = ss_rpmB(ord);

p_linA = polyfit(ss_u, ss_rpmA, 1);
p_linB = polyfit(ss_u, ss_rpmB, 1);
R2_A = 1 - sum((ss_rpmA - polyval(p_linA, ss_u)).^2) / sum((ss_rpmA - mean(ss_rpmA)).^2);
R2_B = 1 - sum((ss_rpmB - polyval(p_linB, ss_u)).^2) / sum((ss_rpmB - mean(ss_rpmB)).^2);

u_grid = linspace(0, 90, 300);
yA_line = Kdc_A_rpm * max(0, u_grid - udz_A);
yB_line = Kdc_B_rpm * max(0, u_grid - udz_B);

fig = figure('Units', 'centimeters', 'Position', [2 2 15.5 10.0], 'Visible', 'off');
ax = axes(fig); hold(ax, 'on');
h1 = plot(ax, ss_u, ss_rpmA, 'o', 'Color', C_BLUE, 'MarkerFaceColor', C_BLUE, 'MarkerSize', 6);
h2 = plot(ax, u_grid, yA_line, '-', 'Color', C_BLUE, 'LineWidth', 1.5);
h3 = plot(ax, ss_u, ss_rpmB, 's', 'Color', C_RED, 'MarkerFaceColor', 'w', 'MarkerSize', 6.5, 'LineWidth', 1.3);
h4 = plot(ax, u_grid, yB_line, '--', 'Color', C_RED, 'LineWidth', 1.5);

[~, idx85] = max(ss_u);
plot(ax, [85 85], [ss_rpmA(idx85) ss_rpmB(idx85)], 'k-', 'LineWidth', 1.1);
text(ax, 83, 294, '\Delta = +5,6%', 'HorizontalAlignment', 'right', 'FontWeight', 'bold', 'FontSize', 10);

xlim(ax, [0 92]); ylim(ax, [0 330]);
xlabel(ax, 'Độ rộng xung PWM (%)', 'FontSize', 11.5);
ylabel(ax, 'Tốc độ xác lập (RPM)', 'FontSize', 11.5);
legend(ax, [h1 h2 h3 h4], { ...
    'Đo thực nghiệm – Động cơ A', ...
    sprintf('Mô hình A: K_{dc}=3,453 RPM/%%, u_{dz}=2,40%% (R^2=%.4f)', R2_A), ...
    'Đo thực nghiệm – Động cơ B', ...
    sprintf('Mô hình B: K_{dc}=3,601 RPM/%%, u_{dz}=1,68%% (R^2=%.4f)', R2_B)}, ...
    'Location', 'northwest', 'Box', 'on');
style_axes(ax);

ax_in = axes(fig, 'Position', [0.57, 0.19, 0.31, 0.28]); hold(ax_in, 'on');
plot(ax_in, u_grid, yA_line, '-',  'Color', C_BLUE, 'LineWidth', 1.4);
plot(ax_in, u_grid, yB_line, '--', 'Color', C_RED,  'LineWidth', 1.4);
plot(ax_in, udz_A, 0, 'o', 'Color', C_BLUE, 'MarkerFaceColor', C_BLUE, 'MarkerSize', 5);
plot(ax_in, udz_B, 0, 's', 'Color', C_RED,  'MarkerFaceColor', C_RED,  'MarkerSize', 5);
text(ax_in, udz_A + 0.2, 2.5, 'u_{dz,A}=2,40%', 'Color', C_BLUE, 'FontSize', 9.5);
text(ax_in, udz_B - 0.1, 12.5, 'u_{dz,B}=1,68%', 'Color', C_RED,  'FontSize', 9.5);
xlim(ax_in, [0 7]); ylim(ax_in, [0 20]);
title(ax_in, 'Phóng to vùng chết u_{dz}', 'FontSize', 9.5, 'FontWeight', 'normal');
xlabel(ax_in, 'PWM (%)', 'FontSize', 9); ylabel(ax_in, 'RPM', 'FontSize', 9);
style_axes(ax_in); ax_in.FontSize = 9;

fname = 'Hình 5.4. Đặc tính tốc độ xác lập và vùng chết của hai động cơ.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 5.5: Kiểm chứng mô hình Validation Compare (20 – 29s)
% --------------------------------------------------------------------------
fig = figure('Units', 'centimeters', 'Position', [2 2 16 10.5], 'Visible', 'off');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile; hold(ax1, 'on');
patch(ax1, [23.0 23.7 23.7 23.0], [0 0 1.1 1.1], [1.0 0.92 0.90], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
patch(ax1, [27.5 28.2 28.2 27.5], [0 0 1.1 1.1], [1.0 0.92 0.90], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
h_m = plot(ax1, t_id(m_val), vA_id(m_val), '-',  'Color', C_BLACK,  'LineWidth', 1.4);
h_1 = plot(ax1, t_id(m_val), vA_m1(m_val), '--', 'Color', C_BLUE,   'LineWidth', 1.45);
h_2 = plot(ax1, t_id(m_val), vA_m2(m_val), ':',  'Color', C_ORANGE, 'LineWidth', 1.5);
text(ax1, 23.35, 0.96, {'Giảm tốc', '85\rightarrow35%'}, 'HorizontalAlignment', 'center', 'FontSize', 9.5, 'Color', C_RED);
text(ax1, 27.85, 0.96, {'Giảm tốc', '65\rightarrow20%'}, 'HorizontalAlignment', 'center', 'FontSize', 9.5, 'Color', C_RED);
xlim(ax1, [20 29]); ylim(ax1, [0.10 1.08]);
ylabel(ax1, 'Vận tốc A (m/s)', 'FontSize', 11.5);
title(ax1, sprintf('Động cơ A – Độ khớp kiểm chứng (Validation Fit = %.2f%%)', fitA_val), 'FontSize', 11);
legend(ax1, [h_m h_1 h_2], {'Thực đo (M/T)', 'Mô hình bậc 1 G_{1}(s) + u_{dz}', 'Mô hình bậc 2 G_{2}(s) (trùng G_{1})'}, ...
    'Location', 'southwest', 'Box', 'on');
style_axes(ax1);

ax2 = nexttile; hold(ax2, 'on');
patch(ax2, [23.0 23.8 23.8 23.0], [0 0 1.15 1.15], [1.0 0.92 0.90], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
patch(ax2, [27.5 28.3 28.3 27.5], [0 0 1.15 1.15], [1.0 0.92 0.90], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
h_mb = plot(ax2, t_id(m_val), vB_id(m_val), '-',  'Color', C_BLACK,  'LineWidth', 1.4);
h_1b = plot(ax2, t_id(m_val), vB_m1(m_val), '--', 'Color', C_RED,    'LineWidth', 1.45);
h_2b = plot(ax2, t_id(m_val), vB_m2(m_val), ':',  'Color', C_ORANGE, 'LineWidth', 1.5);
xlim(ax2, [20 29]); ylim(ax2, [0.10 1.12]);
xlabel(ax2, 'Thời gian (s)', 'FontSize', 11.5);
ylabel(ax2, 'Vận tốc B (m/s)', 'FontSize', 11.5);
title(ax2, sprintf('Động cơ B – Độ khớp kiểm chứng (Validation Fit = %.2f%%)', fitB_val), 'FontSize', 11);
legend(ax2, [h_mb h_1b h_2b], {'Thực đo (M/T)', 'Mô hình bậc 1 G_{1}(s) + u_{dz}', 'Mô hình bậc 2 G_{2}(s) (trùng G_{1})'}, ...
    'Location', 'southwest', 'Box', 'on');
style_axes(ax2);
linkaxes([ax1 ax2], 'x');

fname = 'Hình 5.5. Kiểm chứng mô hình bậc 1 và bậc 2 trên tập Validation.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.1: Biểu đồ Bode vòng hở L(jω) có xét trễ ẩn T_d = 8–10 ms
% --------------------------------------------------------------------------
w = logspace(0, 2.6, 800);
w_gc_sel = 35;
w_gc_hi  = 80;
Td_nom   = 0.009;
Td_lo    = 0.008;
Td_hi    = 0.010;

L_sel_mag = 20 * log10(w_gc_sel ./ w);
L_hi_mag  = 20 * log10(w_gc_hi  ./ w);

ph_nom = -90 - (w * Td_nom) * 180 / pi;
ph_lo  = -90 - (w * Td_lo)  * 180 / pi;
ph_hi  = -90 - (w * Td_hi)  * 180 / pi;

PM_sel_nom = 90 - w_gc_sel * Td_nom * 180 / pi;
PM_sel_10  = 90 - w_gc_sel * Td_hi  * 180 / pi;
PM_sel_8   = 90 - w_gc_sel * Td_lo  * 180 / pi;
PM_hi_10   = 90 - w_gc_hi  * Td_hi  * 180 / pi;

fig = figure('Units', 'centimeters', 'Position', [2 2 15.5 11.0], 'Visible', 'off');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile; hold(ax1, 'on');
set(ax1, 'XScale', 'log');
h1 = plot(ax1, w, L_sel_mag, '-',  'Color', C_BLUE, 'LineWidth', 1.6);
h2 = plot(ax1, w, L_hi_mag,  '--', 'Color', C_RED,  'LineWidth', 1.4);
yline(ax1, 0, 'k-', 'LineWidth', 0.9);
plot(ax1, w_gc_sel, 0, 'o', 'Color', C_BLUE, 'MarkerFaceColor', C_BLUE, 'MarkerSize', 6.5);
plot(ax1, w_gc_hi,  0, 's', 'Color', C_RED,  'MarkerFaceColor', C_RED,  'MarkerSize', 6);
xline(ax1, w_gc_sel, ':', 'Color', C_BLUE, 'LineWidth', 1.2);
xline(ax1, w_gc_hi,  ':', 'Color', C_RED,  'LineWidth', 1.1);
text(ax1, w_gc_sel*0.92, -10, '\omega_{gc} = 35 rad/s', 'HorizontalAlignment', 'right', ...
    'Color', C_BLUE, 'FontWeight', 'bold', 'FontSize', 10.5);
text(ax1, w_gc_hi*1.08, 8, '\omega_{gc} = 80 rad/s', 'HorizontalAlignment', 'left', ...
    'Color', C_RED, 'FontSize', 10);
xlim(ax1, [1 400]); ylim(ax1, [-30 40]);
ylabel(ax1, 'Biên độ (dB)', 'FontSize', 11.5);
legend(ax1, [h1 h2], {'Thiết kế chọn (\omega_{gc} = 35 rad/s)', 'Đối chiếu ép băng thông (\omega_{gc} = 80 rad/s)'}, ...
    'Location', 'southwest', 'Box', 'on');
style_axes(ax1);

ax2 = nexttile; hold(ax2, 'on');
set(ax2, 'XScale', 'log');
fill(ax2, [w fliplr(w)], [ph_lo fliplr(ph_hi)], [0.80 0.89 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.65);
p_nom = plot(ax2, w, ph_nom, '-',  'Color', C_BLUE, 'LineWidth', 1.6);
p_hi  = plot(ax2, w, ph_hi,  '--', 'Color', C_GRAY, 'LineWidth', 1.0);
yline(ax2, -180, 'k--', 'LineWidth', 1.1);
xline(ax2, w_gc_sel, ':', 'Color', C_BLUE, 'LineWidth', 1.2);
xline(ax2, w_gc_hi,  ':', 'Color', C_RED,  'LineWidth', 1.1);

ph_at_35 = -180 + PM_sel_nom;
ph_at_80 = -180 + PM_hi_10;
plot(ax2, [w_gc_sel w_gc_sel], [-180 ph_at_35], '-', 'Color', C_GREEN, 'LineWidth', 2.4);
plot(ax2, w_gc_sel, ph_at_35, 'o', 'Color', C_GREEN, 'MarkerFaceColor', C_GREEN, 'MarkerSize', 6);
text(ax2, w_gc_sel*0.88, -145, sprintf('PM = %.1f° (T_d=9ms)\n[%.1f° – %.1f° với 8–10ms]', ...
    PM_sel_nom, PM_sel_10, PM_sel_8), 'HorizontalAlignment', 'right', ...
    'Color', C_GREEN, 'FontWeight', 'bold', 'FontSize', 10);

plot(ax2, [w_gc_hi w_gc_hi], [-180 ph_at_80], '-', 'Color', C_RED, 'LineWidth', 2.0);
plot(ax2, w_gc_hi, ph_at_80, 's', 'Color', C_RED, 'MarkerFaceColor', C_RED, 'MarkerSize', 5.5);
text(ax2, w_gc_hi*1.08, -158, sprintf('PM = %.1f°\n(T_d=10ms)', PM_hi_10), ...
    'HorizontalAlignment', 'left', 'Color', C_RED, 'FontSize', 9.5);

xlim(ax2, [1 400]); ylim(ax2, [-230 -80]);
yticks(ax2, [-225 -180 -135 -90]);
xlabel(ax2, 'Tần số góc \omega (rad/s)', 'FontSize', 11.5);
ylabel(ax2, 'Góc pha (độ)', 'FontSize', 11.5);
legend(ax2, [p_nom p_hi], {'Pha với trễ ẩn T_d = 9ms (dải tô: 8 – 10ms)', 'Biên xấu nhất T_d = 10ms'}, ...
    'Location', 'southwest', 'Box', 'on');
style_axes(ax2);
linkaxes([ax1 ax2], 'x');

fname = 'Hình 6.1. Biểu đồ Bode vòng hở có xét trễ ẩn.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.3: Đối chứng pha xác lập và phổ FFT trước (M/T 4x) và sau (M/T 1x)
% --------------------------------------------------------------------------
t_cl  = double(T_cl1x.seq) * Ts;
va_4x = double(T_cl4x.va);   va_1x = double(T_cl1x.va);
vr_cl = double(T_cl1x.vref);

win_defs = { ...
    '0,3 m/s', [4.90, 5.895], 30.4; ...
    '0,5 m/s', [3.60, 4.595], 16.0; ...
    '0,7 m/s', [7.50, 8.495], 62.4};

fig = figure('Units', 'centimeters', 'Position', [2 2 16.5 11.5], 'Visible', 'off');
tl = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:3
    lbl   = win_defs{k, 1};
    t_win = win_defs{k, 2};
    f_th  = win_defs{k, 3};

    m_w = (t_cl >= t_win(1)) & (t_cl <= t_win(2));
    tw  = t_cl(m_w) - t_win(1);
    e4  = (va_4x(m_w) - mean(va_4x(m_w))) * 1000;
    e1  = (va_1x(m_w) - mean(va_1x(m_w))) * 1000;
    rms4 = sqrt(mean(e4.^2));
    rms1 = sqrt(mean(e1.^2));

    ax_t = nexttile(k); hold(ax_t, 'on');
    plot(ax_t, tw, e4, '-',  'Color', C_RED,  'LineWidth', 1.15);
    plot(ax_t, tw, e1, '-',  'Color', C_BLUE, 'LineWidth', 1.25);
    xlim(ax_t, [0 1.0]); ylim(ax_t, [-10 10]);
    title(ax_t, sprintf('v_{ref} = %s', lbl), 'FontSize', 11);
    if k == 1, ylabel(ax_t, 'Dao động vận tốc (mm/s)', 'FontSize', 11); end
    xlabel(ax_t, 'Thời gian tương đối (s)', 'FontSize', 10.5);
    text(ax_t, 0.04, 8.2, sprintf('4x: RMS=%.2f', rms4), 'Color', C_RED,  'FontSize', 9.2, 'FontWeight', 'bold');
    text(ax_t, 0.04, 6.0, sprintf('1x: RMS=%.2f', rms1), 'Color', C_BLUE, 'FontSize', 9.2, 'FontWeight', 'bold');
    if k == 3
        legend(ax_t, {'Trước (M/T 4x)', 'Sau (M/T 1x)'}, 'Location', 'southeast', 'Box', 'on', 'FontSize', 9.5);
    end
    style_axes(ax_t);

    ax_f = nexttile(k + 3); hold(ax_f, 'on');
    [f_ax, A4] = calc_single_sided_fft(e4, fs, 2048);
    [~,    A1] = calc_single_sided_fft(e1, fs, 2048);
    plot(ax_f, f_ax, A4, '-',  'Color', C_RED,  'LineWidth', 1.25);
    plot(ax_f, f_ax, A1, '-',  'Color', C_BLUE, 'LineWidth', 1.25);
    xline(ax_f, f_th, '--', 'Color', C_GRAY, 'LineWidth', 1.0);

    m_pk = (f_ax >= f_th - 3) & (f_ax <= f_th + 3);
    [pk_val, idx_local] = max(A4(m_pk));
    f_sub = f_ax(m_pk);
    f_pk  = f_sub(idx_local);
    plot(ax_f, f_pk, pk_val, 'o', 'Color', C_RED, 'MarkerFaceColor', C_RED, 'MarkerSize', 5);
    text(ax_f, min(f_pk + 3, 66), pk_val + 0.25, sprintf('%.1f Hz', f_pk), ...
        'Color', C_RED, 'FontWeight', 'bold', 'FontSize', 9.5);

    xlim(ax_f, [0 100]); ylim(ax_f, [0 4.6]);
    xlabel(ax_f, 'Tần số (Hz)', 'FontSize', 10.5);
    if k == 1, ylabel(ax_f, 'Biên độ phổ FFT (mm/s)', 'FontSize', 11); end
    style_axes(ax_f);
end

fname = 'Hình 6.3. So sánh pha xác lập và phổ FFT giữa M-T 4x và M-T 1x.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.4: Đáp ứng vòng kín 10s trên gá đỡ sau cải tiến M/T 1x
% --------------------------------------------------------------------------
vb_1x   = double(T_cl1x.vb);
pwma_1x = double(T_cl1x.pwm_a);
pwmb_1x = double(T_cl1x.pwm_b);
diff_1x = (va_1x - vb_1x) * 1000;

fig = figure('Units', 'centimeters', 'Position', [2 2 16 13.0], 'Visible', 'off');
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile; hold(ax1, 'on');
h_r = plot(ax1, t_cl, vr_cl, '--', 'Color', C_BLACK, 'LineWidth', 1.3);
h_a = plot(ax1, t_cl, va_1x, '-',  'Color', C_BLUE,  'LineWidth', 1.35);
h_b = plot(ax1, t_cl, vb_1x, '-.', 'Color', C_RED,   'LineWidth', 1.25);
xlim(ax1, [0 10]); ylim(ax1, [-0.03 0.78]);
ylabel(ax1, 'Vận tốc (m/s)', 'FontSize', 11.5);
legend(ax1, [h_r h_a h_b], {'Vận tốc đặt v_{ref}', 'Động cơ A (v_A)', 'Động cơ B (v_B)'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_axes(ax1);

ax2 = nexttile; hold(ax2, 'on');
plot(ax2, t_cl, pwma_1x, '-',  'Color', C_BLUE, 'LineWidth', 1.3);
plot(ax2, t_cl, pwmb_1x, '--', 'Color', C_RED,  'LineWidth', 1.3);
xlim(ax2, [0 10]); ylim(ax2, [-5 108]);
ylabel(ax2, 'Độ rộng xung PWM (%)', 'FontSize', 11.5);
legend(ax2, {'PWM Động cơ A', 'PWM Động cơ B'}, 'Location', 'northeast', 'Box', 'on');
style_axes(ax2);

ax3 = nexttile; hold(ax3, 'on');
patch(ax3, [0 10 10 0], [-4 -4 4 4], [0.88 0.96 0.92], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
plot(ax3, t_cl, diff_1x, '-', 'Color', C_GREEN, 'LineWidth', 1.2);
yline(ax3,  4, '--', 'Color', C_GRAY, 'LineWidth', 0.9);
yline(ax3, -4, '--', 'Color', C_GRAY, 'LineWidth', 0.9);
text(ax3, 8.7, 13, 'Pha xác lập: |v_A - v_B| \leq 4 mm/s', 'HorizontalAlignment', 'right', ...
    'Color', C_GREEN, 'FontWeight', 'bold', 'FontSize', 10);
xlim(ax3, [0 10]); ylim(ax3, [-25 25]);
xlabel(ax3, 'Thời gian (s)', 'FontSize', 11.5);
ylabel(ax3, 'v_A - v_B (mm/s)', 'FontSize', 11.5);
style_axes(ax3);
linkaxes([ax1 ax2 ax3], 'x');

fname = 'Hình 6.4. Đáp ứng vòng kín 10s trên gá đỡ sau cải tiến M-T 1x.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.5: So sánh đáp ứng trên mặt bạt sa bàn 0 kg, 1 kg và 2 kg
% --------------------------------------------------------------------------
t_m   = double(T_mat0.seq) * Ts;
vr_m  = double(T_mat0.vref);
va_m0 = double(T_mat0.va);    vb_m0 = double(T_mat0.vb);
pa_m0 = double(T_mat0.pwm_a); pb_m0 = double(T_mat0.pwm_b);
va_m1 = double(T_mat1.va);    vb_m1 = double(T_mat1.vb);
pa_m1 = double(T_mat1.pwm_a); pb_m1 = double(T_mat1.pwm_b);
va_m2 = double(T_mat2.va);    vb_m2 = double(T_mat2.vb);
pa_m2 = double(T_mat2.pwm_a); pb_m2 = double(T_mat2.pwm_b);

m_brk = (t_m >= 8.50);
d_a0 = sum(max(va_m0(m_brk), 0)) * Ts * 1000; d_b0 = sum(max(vb_m0(m_brk), 0)) * Ts * 1000;
d_a1 = sum(max(va_m1(m_brk), 0)) * Ts * 1000; d_b1 = sum(max(vb_m1(m_brk), 0)) * Ts * 1000;
d_a2 = sum(max(va_m2(m_brk), 0)) * Ts * 1000; d_b2 = sum(max(vb_m2(m_brk), 0)) * Ts * 1000;

fig = figure('Units', 'centimeters', 'Position', [2 2 16.5 13.8], 'Visible', 'off');
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile([1 2]); hold(ax1, 'on');
h_r = plot(ax1, t_m, vr_m,  '--', 'Color', C_BLACK,  'LineWidth', 1.3);
h_0 = plot(ax1, t_m, va_m0, '-',  'Color', C_BLUE,   'LineWidth', 1.35);
h_1 = plot(ax1, t_m, va_m1, '-',  'Color', C_ORANGE, 'LineWidth', 1.25);
h_2 = plot(ax1, t_m, va_m2, ':',  'Color', C_RED,    'LineWidth', 1.35);
xlim(ax1, [0 10]); ylim(ax1, [-0.03 0.78]);
ylabel(ax1, 'Vận tốc (m/s)', 'FontSize', 11.5);
legend(ax1, [h_r h_0 h_1 h_2], {'v_{ref}', 'Bạt 0 kg (1,3 kg)', 'Bạt +1 kg (2,3 kg)', 'Bạt +2 kg (3,3 kg)'}, ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'Box', 'off');
style_axes(ax1);

ax2 = nexttile([1 2]); hold(ax2, 'on');
plot(ax2, t_m, pa_m0, '-',  'Color', C_BLUE,   'LineWidth', 1.3);
plot(ax2, t_m, pa_m1, '-',  'Color', C_ORANGE, 'LineWidth', 1.25);
plot(ax2, t_m, pa_m2, '--', 'Color', C_RED,    'LineWidth', 1.3);
xlim(ax2, [0 10]); ylim(ax2, [-5 108]);
xlabel(ax2, 'Thời gian (s)', 'FontSize', 11);
ylabel(ax2, 'PWM Động cơ A (%)', 'FontSize', 11.5);
legend(ax2, {'0 kg', '+1 kg', '+2 kg'}, 'Location', 'northeast', 'Orientation', 'horizontal', 'Box', 'on');
style_axes(ax2);

ax3 = nexttile; hold(ax3, 'on');
plot(ax3, t_m, vr_m,  '--', 'Color', C_BLACK,  'LineWidth', 1.35);
plot(ax3, t_m, va_m0, '-',  'Color', C_BLUE,   'LineWidth', 1.35);
plot(ax3, t_m, va_m1, '-',  'Color', C_ORANGE, 'LineWidth', 1.3);
plot(ax3, t_m, va_m2, ':',  'Color', C_RED,    'LineWidth', 1.35);
xlim(ax3, [0.25 1.25]); ylim(ax3, [-0.02 0.56]);
title(ax3, 'Phóng to khởi động S-curve (0,3 – 1,1s)', 'FontSize', 10.5);
xlabel(ax3, 'Thời gian (s)', 'FontSize', 11);
ylabel(ax3, 'Vận tốc (m/s)', 'FontSize', 11);
style_axes(ax3);

ax4 = nexttile; hold(ax4, 'on');
plot(ax4, t_m, vr_m,  '--', 'Color', C_BLACK,  'LineWidth', 1.2);
plot(ax4, t_m, va_m0, '-',  'Color', C_BLUE,   'LineWidth', 1.35);
plot(ax4, t_m, va_m1, '-',  'Color', C_ORANGE, 'LineWidth', 1.35);
plot(ax4, t_m, va_m2, ':',  'Color', C_RED,    'LineWidth', 1.4);
xlim(ax4, [8.45 8.90]); ylim(ax4, [-0.02 0.75]);
title(ax4, 'Phóng to phanh Short Brake (t = 8,5s)', 'FontSize', 10.5);
xlabel(ax4, 'Thời gian (s)', 'FontSize', 11);
ylabel(ax4, 'Vận tốc (m/s)', 'FontSize', 11);
text(ax4, 8.63, 0.48, sprintf('d_{stop} A / B (mm):\n0 kg: %.1f / %.1f\n1 kg: %.1f / %.1f\n2 kg: %.1f / %.1f', ...
    d_a0, d_b0, d_a1, d_b1, d_a2, d_b2), 'FontSize', 8.8, 'BackgroundColor', 'w', 'EdgeColor', C_GRAY);
style_axes(ax4);

fname = 'Hình 6.5. Đáp ứng vận tốc và PWM trên mặt bạt sa bàn tại ba mức tải.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);


%% =========================================================================
%  PHẦN C: 6 HÌNH BỔ SUNG CỦNG CỐ PHÂN TÍCH & CHẨN ĐOÁN
% =========================================================================

%% -------------------------------------------------------------------------
%  HÌNH 5.1b: Sai số lượng tử hóa Method M (1x, 4x) vs M/T
% --------------------------------------------------------------------------
v_rng = linspace(0.10, 0.80, 300);
err_M1x = 100 ./ ((v_rng ./ C_wheel) * N_1x * Ts);
err_M4x = 100 ./ ((v_rng ./ C_wheel) * N_4x * Ts);
dt_mt_us = max(Ts * 1e6, 1e6 ./ ((v_rng ./ C_wheel) * N_1x));
err_MT   = 100 * (1.0 ./ dt_mt_us);

fig = figure('Units', 'centimeters', 'Position', [2 2 15.0 9.0], 'Visible', 'off');
ax = axes(fig); hold(ax, 'on');
set(ax, 'YScale', 'log');
h1 = plot(ax, v_rng, err_M1x, '--', 'Color', C_RED,    'LineWidth', 1.6);
h2 = plot(ax, v_rng, err_M4x, '-.', 'Color', C_ORANGE, 'LineWidth', 1.6);
h3 = plot(ax, v_rng, err_MT,  '-',  'Color', C_BLUE,   'LineWidth', 1.8);
xline(ax, 0.30, ':', 'Color', C_GRAY, 'LineWidth', 1.1);
plot(ax, 0.30, 41.3,  'o', 'Color', C_RED,    'MarkerFaceColor', C_RED,    'MarkerSize', 6);
plot(ax, 0.30, 10.3,  's', 'Color', C_ORANGE, 'MarkerFaceColor', C_ORANGE, 'MarkerSize', 6);
plot(ax, 0.30, 0.024, '^', 'Color', C_BLUE,   'MarkerFaceColor', C_BLUE,   'MarkerSize', 6);
text(ax, 0.315, 45,    '41,3% (M 1x)', 'Color', C_RED,    'FontWeight', 'bold', 'FontSize', 10);
text(ax, 0.315, 11.5,  '10,3% (M 4x)', 'Color', C_ORANGE, 'FontWeight', 'bold', 'FontSize', 10);
text(ax, 0.315, 0.036, '0,024% (M/T)', 'Color', C_BLUE,   'FontWeight', 'bold', 'FontSize', 10);
xlim(ax, [0.10 0.80]); ylim(ax, [0.008 200]);
xlabel(ax, 'Vận tốc dài bánh xe (m/s)', 'FontSize', 11.5);
ylabel(ax, 'Sai số lượng tử hóa (%)', 'FontSize', 11.5);
legend(ax, [h1 h2 h3], {'Đếm xung Method M – 1x (330 xung/vòng)', ...
    'Đếm xung Method M – 4x (1320 xung/vòng)', 'Đo xung – thời gian M/T (timer 1 \mus)'}, ...
    'Location', 'northeast', 'Box', 'on');
style_axes(ax);

fname = 'Hình 5.1b. So sánh sai số lượng tử hóa giữa Method M và M-T.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 5.5b: Phóng to bất đối xứng hằng số thời gian τ_63%
% --------------------------------------------------------------------------
fig = figure('Units', 'centimeters', 'Position', [2 2 16.0 8.5], 'Visible', 'off');
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

motors_info = {'Động cơ A', rpmA, tau_A*1000; 'Động cơ B', rpmB, tau_B*1000};
for m_idx = 1:2
    ax = nexttile; hold(ax, 'on');
    y_rpm = motors_info{m_idx, 2};

    [ts1, yn1, t63_up]  = extract_norm_step(t_id, y_rpm, 20.5, 0.60);
    [ts2, yn2, t63_brk] = extract_norm_step(t_id, y_rpm, 16.0, 0.60);
    [ts3, yn3, t63_dn]  = extract_norm_step(t_id, y_rpm, 23.0, 0.60);

    h1 = plot(ax, ts1*1000, yn1*100, '-',  'Color', C_BLUE,  'LineWidth', 1.6);
    h2 = plot(ax, ts2*1000, yn2*100, '-.', 'Color', C_GREEN, 'LineWidth', 1.6);
    h3 = plot(ax, ts3*1000, yn3*100, '--', 'Color', C_RED,   'LineWidth', 1.6);
    yline(ax, 63.2, ':', 'Color', C_BLACK, 'LineWidth', 1.1);
    plot(ax, [t63_up t63_brk t63_dn], [63.2 63.2 63.2], 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 5.5);

    xlim(ax, [0 500]); ylim(ax, [-5 110]);
    title(ax, motors_info{m_idx, 1}, 'FontSize', 11.5);
    xlabel(ax, 'Thời gian sau bước nhảy (ms)', 'FontSize', 11);
    if m_idx == 1, ylabel(ax, 'Đáp ứng chuẩn hóa (%)', 'FontSize', 11.5); end
    legend(ax, [h1 h2 h3], { ...
        sprintf('Tăng tốc 50\\rightarrow85%% (\\tau_{63}=%.0f ms)', t63_up), ...
        sprintf('Phanh cứng 70\\rightarrow0%% (\\tau_{63}=%.0f ms)', t63_brk), ...
        sprintf('Giảm tốc 85\\rightarrow35%% (\\tau_{63}=%.0f ms)', t63_dn)}, ...
        'Location', 'southeast', 'Box', 'on', 'FontSize', 9.5);
    style_axes(ax);
end

fname = 'Hình 5.5b. Bất đối xứng hằng số thời gian khi tăng tốc, giảm tốc và phanh.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.1b: S-curve (v, a) & So sánh mô phỏng ω_gc=35 vs 80 rad/s
% --------------------------------------------------------------------------
t_sc = linspace(0, 1.0, 400);
tau_sc = min(max(t_sc / 0.8, 0), 1);
v_sc = 0.5 * (3*tau_sc.^2 - 2*tau_sc.^3);
a_sc = (t_sc <= 0.8) .* (0.5 / 0.8 * (6*tau_sc - 6*tau_sc.^2));

t_sim = (0:Ts:0.40)';
vref_s = 0.5 * ones(size(t_sim)); vref_s(t_sim >= 0.05) = 0.7;
[v_35, u_35] = sim_cl_2dof_pi(vref_s, Kdc_A, tau_A, udz_A, 227.0, 2978.0, 85.09, udz_A, 0.035, Ts, 2, true);
[v_80, u_80] = sim_cl_2dof_pi(vref_s, Kdc_A, tau_A, udz_A, 518.7, 6807.2, 85.09, udz_A, 0.035, Ts, 2, true);
[~,    u_80_nosat] = sim_cl_2dof_pi(vref_s, Kdc_A, tau_A, udz_A, 518.7, 6807.2, 85.09, udz_A, 0.035, Ts, 2, false);

fig = figure('Units', 'centimeters', 'Position', [2 2 16.5 9.0], 'Visible', 'off');
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile; hold(ax1, 'on');
yyaxis(ax1, 'left');
h_v = plot(ax1, t_sc, v_sc, '-', 'Color', C_BLUE, 'LineWidth', 1.7);
ylabel(ax1, 'Vận tốc đặt v_{ref} (m/s)', 'FontSize', 11); ylim(ax1, [0 0.60]);
ax1.YColor = C_BLUE;

yyaxis(ax1, 'right');
h_a = plot(ax1, t_sc, a_sc, '--', 'Color', C_RED, 'LineWidth', 1.6);
h_l = yline(ax1, 1.0, ':', 'Color', C_BLACK, 'LineWidth', 1.2);
text(ax1, 0.40, 0.96, 'a_{max} = 0,9375 m/s^2 \leq 1,0', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Color', C_RED, 'FontSize', 9.5, 'FontWeight', 'bold');
ylabel(ax1, 'Gia tốc a_{ref} (m/s^2)', 'FontSize', 11); ylim(ax1, [0 1.25]);
ax1.YColor = C_RED;

xlim(ax1, [0 1.0]); xlabel(ax1, 'Thời gian (s)', 'FontSize', 11);
title(ax1, 'Quỹ đạo khởi động S-curve bậc 3', 'FontSize', 11);
legend(ax1, [h_v h_a h_l], {'Vận tốc v_{ref}', 'Gia tốc a_{ref}', 'Giới hạn 1,0 m/s^2'}, ...
    'Location', 'southeast', 'Box', 'on', 'FontSize', 9);
grid(ax1, 'on'); box(ax1, 'on');

ax2 = nexttile; hold(ax2, 'on');
yyaxis(ax2, 'left');
p_r  = plot(ax2, t_sim, vref_s, ':',  'Color', C_BLACK, 'LineWidth', 1.2);
p_35 = plot(ax2, t_sim, v_35,   '-',  'Color', C_BLUE,  'LineWidth', 1.6);
p_80 = plot(ax2, t_sim, v_80,   '--', 'Color', C_RED,   'LineWidth', 1.5);
ylabel(ax2, 'Vận tốc mô phỏng (m/s)', 'FontSize', 11); ylim(ax2, [0.47 0.76]);
ax2.YColor = C_BLUE;

yyaxis(ax2, 'right');
plot(ax2, t_sim, u_35, '-',  'Color', C_GREEN,  'LineWidth', 1.1);
plot(ax2, t_sim, u_80, '-.', 'Color', C_ORANGE, 'LineWidth', 1.2);
plot(ax2, 0.05, max(u_80_nosat), 'd', 'Color', C_RED, 'MarkerFaceColor', C_RED, 'MarkerSize', 5.5);
text(ax2, 0.075, 162, 'u_{đỉnh} = 165,7% (bão hòa)', 'Color', C_RED, 'FontSize', 9.2, 'FontWeight', 'bold');
yline(ax2, 100, ':', 'Color', C_ORANGE, 'LineWidth', 1.0);
ylabel(ax2, 'Tín hiệu PWM (%)', 'FontSize', 11); ylim(ax2, [35 178]);
ax2.YColor = C_ORANGE;

xlim(ax2, [0 0.35]); xlabel(ax2, 'Thời gian (s)', 'FontSize', 11);
title(ax2, 'Đối chứng \omega_{gc} = 35 vs 80 rad/s (T_d = 10ms)', 'FontSize', 11);
legend(ax2, [p_r p_35 p_80], {'v_{ref}', 'v (\omega_{gc}=35, êm)', 'v (\omega_{gc}=80, vọt lố)'}, ...
    'Location', 'southeast', 'Box', 'on', 'FontSize', 9);
grid(ax2, 'on'); box(ax2, 'on');

fname = 'Hình 6.1b. Quỹ đạo khởi động S-curve và đối chứng tần số cắt biên.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.3a: Phóng to gai mất 2 xung ở M/T 4x
% --------------------------------------------------------------------------
vb_4x   = double(T_cl4x.vb);
pwma_4x = double(T_cl4x.pwm_a);

fig = figure('Units', 'centimeters', 'Position', [2 2 16.0 10.5], 'Visible', 'off');
tl = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

spikes = {[2.90 3.05], 2.960, 0.70, 'Gai lần 1 tại t = 2,960s (v_{ref} = 0,7 m/s)'; ...
          [6.66 6.81], 6.725, 0.50, 'Gai lần 2 tại t = 6,725s (v_{ref} = 0,5 m/s)'};

for s = 1:2
    tw   = spikes{s, 1};
    vset = spikes{s, 3};
    ttl  = spikes{s, 4};

    ax_v = nexttile(s); hold(ax_v, 'on');
    m_s = (t_cl >= tw(1)) & (t_cl <= tw(2));
    plot(ax_v, t_cl(m_s), vr_cl(m_s), '--', 'Color', C_BLACK, 'LineWidth', 1.1);
    plot(ax_v, t_cl(m_s), vb_4x(m_s), '-.', 'Color', C_GREEN, 'LineWidth', 1.2);
    plot(ax_v, t_cl(m_s), va_4x(m_s), '-o', 'Color', C_RED,   'LineWidth', 1.35, 'MarkerSize', 3.5);
    yline(ax_v, vset + 0.010, ':', 'Color', C_GRAY, 'LineWidth', 0.9);
    yline(ax_v, vset - 0.010, ':', 'Color', C_GRAY, 'LineWidth', 0.9);

    [~, idx_min] = min(va_4x(m_s)); t_sub = t_cl(m_s); va_sub = va_4x(m_s);
    dv_drop = (va_sub(idx_min) - vset) * 1000;
    [v_pk, idx_max] = max(va_sub);
    os_fake = (v_pk - vset) / 0.2 * 100;
    text(ax_v, t_sub(idx_min) + 0.006, va_sub(idx_min) - 0.003, ...
        sprintf('Gai mất 2 xung:\n%.0f mm/s (1 mẫu)', dv_drop), 'Color', C_RED, 'FontWeight', 'bold', 'FontSize', 9.2);
    text(ax_v, t_sub(idx_max) + 0.005, v_pk + 0.003, ...
        sprintf('Vọt lố giả %.1f%%', os_fake), 'Color', C_BLUE, 'FontWeight', 'bold', 'FontSize', 9.2);

    xlim(ax_v, tw); ylim(ax_v, [vset - 0.08, vset + 0.032]);
    title(ax_v, ttl, 'FontSize', 10.5);
    if s == 1
        ylabel(ax_v, 'Vận tốc (m/s)', 'FontSize', 11);
        legend(ax_v, {'v_{ref}', 'Động cơ B', 'Động cơ A (4x)'}, 'Location', 'southwest', 'Box', 'on', 'FontSize', 9);
    end
    style_axes(ax_v);

    ax_p = nexttile(s + 2); hold(ax_p, 'on');
    plot(ax_p, t_cl(m_s), pwma_4x(m_s), '-s', 'Color', C_BLUE, 'LineWidth', 1.35, 'MarkerSize', 3.5);
    [p_pk, idx_p] = max(pwma_4x(m_s));
    text(ax_p, t_sub(idx_p) + 0.006, p_pk - 1.5, sprintf('PWM kích vọt\n%.1f%%', p_pk), ...
        'Color', C_BLUE, 'FontWeight', 'bold', 'FontSize', 9.5);
    xlim(ax_p, tw);
    xlabel(ax_p, 'Thời gian (s)', 'FontSize', 11);
    if s == 1, ylabel(ax_p, 'PWM Động cơ A (%)', 'FontSize', 11); end
    style_axes(ax_p);
end

fname = 'Hình 6.3a. Phóng to gai mất hai xung ở chế độ M-T 4x.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.5b: Đối chứng tương quan Gá đỡ vs Sàn gạch 30x30 cm vs Mặt bạt
% --------------------------------------------------------------------------
t_fl  = double(T_fl0.seq) * Ts;
vr_fl = double(T_fl0.vref);
va_f0 = double(T_fl0.va); vb_f0 = double(T_fl0.vb);

m_seg = (t_cl >= 3.60) & (t_cl <= 4.595);
ea_b = (va_1x(m_seg) - vr_cl(m_seg)) * 1000; eb_b = (vb_1x(m_seg) - vr_cl(m_seg)) * 1000;
ea_g = (va_f0(m_seg) - vr_fl(m_seg)) * 1000; eb_g = (vb_f0(m_seg) - vr_fl(m_seg)) * 1000;
ea_m = (va_m0(m_seg) - vr_m(m_seg))  * 1000; eb_m = (vb_m0(m_seg) - vr_m(m_seg))  * 1000;

data_3 = { ...
    'Gá đỡ (không tiếp xúc sàn)',      ea_b, eb_b, C_BLUE; ...
    'Sàn gạch 30\times30 cm (0 kg)',   ea_g, eb_g, C_RED; ...
    'Mặt bạt nhựa sa bàn (0 kg)',      ea_m, eb_m, C_GREEN};

fig = figure('Units', 'centimeters', 'Position', [2 2 16.5 11.5], 'Visible', 'off');
tl = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for r_i = 1:3
    ttl = data_3{r_i, 1}; ea_i = data_3{r_i, 2}; eb_i = data_3{r_i, 3}; col_i = data_3{r_i, 4};
    rr = corr_manual(ea_i, eb_i);

    ax_l = nexttile((r_i - 1)*2 + 1); hold(ax_l, 'on');
    plot(ax_l, t_cl(m_seg), ea_i, '-',  'Color', C_BLUE, 'LineWidth', 1.2);
    plot(ax_l, t_cl(m_seg), eb_i, '--', 'Color', C_RED,  'LineWidth', 1.15);
    xlim(ax_l, [3.6 4.6]); ylim(ax_l, [-22 22]);
    title(ax_l, ttl, 'FontSize', 10.5);
    ylabel(ax_l, 'Sai số (mm/s)', 'FontSize', 10.5);
    if r_i == 1
        legend(ax_l, {'e_A', 'e_B'}, 'Location', 'northeast', 'Orientation', 'horizontal', 'Box', 'on', 'FontSize', 9);
    elseif r_i == 3
        xlabel(ax_l, 'Thời gian (s)', 'FontSize', 11);
    end
    style_axes(ax_l);

    ax_r = nexttile((r_i - 1)*2 + 2); hold(ax_r, 'on');
    scatter(ax_r, ea_i, eb_i, 14, col_i, 'filled', 'MarkerFaceAlpha', 0.65);
    xlim(ax_r, [-22 22]); ylim(ax_r, [-22 22]); axis(ax_r, 'square');
    title(ax_r, sprintf('r = %+.2f (max|e| = %.0f mm/s)', rr, max(max(abs(ea_i)), max(abs(eb_i)))), 'FontSize', 10.5);
    ylabel(ax_r, 'e_B (mm/s)', 'FontSize', 10);
    if r_i == 3, xlabel(ax_r, 'e_A (mm/s)', 'FontSize', 10); end
    style_axes(ax_r);
end

fname = 'Hình 6.5b. Đối chứng tương quan sai số giữa gá đỡ, nền gạch và mặt bạt.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

%% -------------------------------------------------------------------------
%  HÌNH 6.6b: Đặc tính Gain Scheduling (ω_gc = 30 rad/s) & d_stop trên bạt
% --------------------------------------------------------------------------
m_vec  = [0, 1, 2];
m_fine = linspace(0, 2, 100);
kpA_f  = 291.0 + 67.6 * m_fine;   kpB_f  = 326.7 + 79.7 * m_fine;
udzA_f = 4.51  + 2.78 * m_fine;   udzB_f = 7.32  + 5.21 * m_fine;
dstop_mat = [57.9 69.8; 65.0 76.9; 72.0 84.0];

fig = figure('Units', 'centimeters', 'Position', [2 2 16.5 8.5], 'Visible', 'off');
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile; hold(ax1, 'on');
yyaxis(ax1, 'left');
h1 = plot(ax1, m_fine, kpA_f, '-',  'Color', C_BLUE, 'LineWidth', 1.7);
h2 = plot(ax1, m_fine, kpB_f, '--', 'Color', C_RED,  'LineWidth', 1.7);
plot(ax1, m_vec, 291.0 + 67.6*m_vec, 'o', 'Color', C_BLUE, 'MarkerFaceColor', C_BLUE, 'MarkerSize', 5.5);
plot(ax1, m_vec, 326.7 + 79.7*m_vec, 's', 'Color', C_RED,  'MarkerFaceColor', C_RED,  'MarkerSize', 5.5);
ylabel(ax1, 'Độ lợi tỉ lệ K_p (%/(m/s))', 'FontSize', 11); ylim(ax1, [250 540]);
ax1.YColor = C_BLUE;

yyaxis(ax1, 'right');
h3 = plot(ax1, m_fine, udzA_f, '-.', 'Color', C_GREEN,  'LineWidth', 1.5);
h4 = plot(ax1, m_fine, udzB_f, ':',  'Color', C_ORANGE, 'LineWidth', 1.7);
ylabel(ax1, 'Vùng chết tĩnh u_{dz} (%)', 'FontSize', 11); ylim(ax1, [0 20]);
ax1.YColor = C_GREEN;

xlim(ax1, [-0.1 2.1]); xticks(ax1, [0 0.5 1.0 1.5 2.0]);
xlabel(ax1, 'Khối lượng hàng m_{load} (kg)', 'FontSize', 11);
title(ax1, 'Luật tham số trên bạt (\omega_{gc} = 30 rad/s)', 'FontSize', 11);
legend(ax1, [h1 h2 h3 h4], {'K_{p,A}', 'K_{p,B}', 'u_{dz,A}', 'u_{dz,B}'}, ...
    'Location', 'northwest', 'Box', 'on', 'FontSize', 9.2);
grid(ax1, 'on'); box(ax1, 'on');

ax2 = nexttile; hold(ax2, 'on');
b = bar(ax2, m_vec, dstop_mat, 'grouped');
b(1).FaceColor = C_BLUE; b(2).FaceColor = C_RED;
for c = 1:2
    x_pos = b(c).XEndPoints;
    y_val = b(c).YEndPoints;
    for r = 1:numel(x_pos)
        text(ax2, x_pos(r), y_val(r) + 2, sprintf('%.1f', y_val(r)), ...
            'HorizontalAlignment', 'center', 'FontSize', 9.2, 'FontWeight', 'bold');
    end
end
xticks(ax2, m_vec); xticklabels(ax2, {'0 kg (1,3 kg)', '1 kg (2,3 kg)', '2 kg (3,3 kg)'});
ylim(ax2, [0 98]);
xlabel(ax2, 'Mức tải hàng (Tổng khối lượng xe)', 'FontSize', 11);
ylabel(ax2, 'Quãng đường phanh d_{stop} (mm)', 'FontSize', 11);
title(ax2, 'Quãng đường hãm Short Brake (0,7 \rightarrow 0 m/s)', 'FontSize', 11);
legend(ax2, {'Động cơ A', 'Động cơ B'}, 'Location', 'northwest', 'Box', 'on', 'FontSize', 9.5);
style_axes(ax2);

fname = 'Hình 6.6b. Đặc tính lập lịch tham số và quãng đường phanh theo tải.svg';
export_fig_svg(fig, fullfile(out_dir, fname));
close(fig); fprintf('Đã xuất figures/%s\n', fname);

fprintf('\nHOÀN TẤT! Đã xuất toàn bộ 16 hình SVG (3 sơ đồ + 7 hình chính + 6 hình bổ sung) vào thư mục figures/.\n');


%% =========================================================================
%  CÁC HÀM PHỤ TRỢ VẼ SƠ ĐỒ VECTOR (KHÔNG PHỤ THUỘC TOOLBOX)
% =========================================================================

function draw_box(ax, x, y, w, h, fcol, ecol, lw, lines, fsz, tcol)
    rx = min(0.25, 2.5 / w);
    ry = min(0.25, 2.5 / h);
    rectangle(ax, 'Position', [x, y, w, h], 'Curvature', [rx ry], ...
        'FaceColor', fcol, 'EdgeColor', ecol, 'LineWidth', lw);
    text(ax, x + w/2, y + h/2, lines, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontSize', fsz, 'Color', tcol);
end

function draw_sum(ax, cx, cy, r, col)
    th = linspace(0, 2*pi, 60);
    patch(ax, cx + r*cos(th), cy + r*sin(th), 'w', 'EdgeColor', col, 'LineWidth', 1.2);
end

function draw_dot(ax, x, y, col)
    plot(ax, x, y, 'o', 'Color', col, 'MarkerFaceColor', col, 'MarkerSize', 3.8);
end

function draw_arrow(ax, x1, y1, x2, y2, col, lw, ls)
    dx = x2 - x1; dy = y2 - y1;
    L = hypot(dx, dy);
    if L < 1e-6, return; end
    ux = dx / L; uy = dy / L;
    px = -uy;    py = ux;
    hlen = min(2.4, 0.45 * L);
    hw   = 0.95;
    % Thân mũi tên dừng nhẹ trước đỉnh nhọn
    plot(ax, [x1, x2 - 0.75*hlen*ux], [y1, y2 - 0.75*hlen*uy], ...
        'LineStyle', ls, 'Color', col, 'LineWidth', lw);
    % Đầu mũi tên vector
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
    hlen = 2.4; hw = 0.95;
    plot(ax, [x1 + 0.75*hlen*ux, x2 - 0.75*hlen*ux], ...
             [y1 + 0.75*hlen*uy, y2 - 0.75*hlen*uy], ...
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

function [xs, ys] = build_square_wave(x_start, x_end, rise_edges, high_w, y_lo, y_hi)
    xs = x_start;
    % Xác định mức khởi tạo tại x_start
    in_hi = any((rise_edges <= x_start) & (x_start < rise_edges + high_w));
    ys = y_lo + double(in_hi) * (y_hi - y_lo);
    % Tạo danh sách sự kiện cạnh lên/xuống trong [x_start, x_end]
    ev_x = []; ev_y = [];
    for i = 1:numel(rise_edges)
        r = rise_edges(i);
        f = r + high_w;
        if r > x_start && r < x_end
            ev_x(end+1) = r; ev_y(end+1) = y_hi; %#ok<AGROW>
        end
        if f > x_start && f < x_end
            ev_x(end+1) = f; ev_y(end+1) = y_lo; %#ok<AGROW>
        end
    end
    [ev_x, ord] = sort(ev_x);
    ev_y = ev_y(ord);
    cur_y = ys;
    for i = 1:numel(ev_x)
        xs(end+1:end+2) = [ev_x(i), ev_x(i)]; %#ok<AGROW>
        ys(end+1:end+2) = [cur_y,   ev_y(i)]; %#ok<AGROW>
        cur_y = ev_y(i);
    end
    xs(end+1) = x_end;
    ys(end+1) = cur_y;
end


%% =========================================================================
%  CÁC HÀM PHỤ TRỢ XỬ LÝ DỮ LIỆU & XUẤT SVG
% =========================================================================

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

function style_axes(ax)
    grid(ax, 'on');
    ax.GridAlpha = 0.25;
    ax.LineWidth = 0.85;
    box(ax, 'on');
end

function y = sim_first_order_zoh(u, Kdc, tau_m, u_dz, Ts)
    u_eff = max(0.0, u - u_dz);
    a = exp(-Ts / tau_m);
    b = Kdc * (1.0 - a);
    N = numel(u);
    y = zeros(N, 1);
    for k = 2:N
        y(k) = a * y(k-1) + b * u_eff(k-1);
    end
end

function y = sim_second_order_zoh(u, Kdc, tau1, tau2, u_dz, Ts)
    u_eff = max(0.0, u - u_dz);
    sub = 10;
    dt  = Ts / sub;
    N   = numel(u);
    y   = zeros(N, 1);
    x1  = 0.0; x2 = 0.0;
    for k = 2:N
        uk = u_eff(k-1);
        for j = 1:sub
            dx1 = (Kdc * uk - x1) / tau1;
            dx2 = (x1 - x2) / tau2;
            x1  = x1 + dt * dx1;
            x2  = x2 + dt * dx2;
        end
        y(k) = x2;
    end
end

function fit_pct = calc_nrmse_fit(y_meas, y_sim)
    fit_pct = (1.0 - norm(y_meas - y_sim) / norm(y_meas - mean(y_meas))) * 100.0;
end

function [f, amp] = calc_single_sided_fft(x, fs, nfft)
    N = numel(x);
    Y = fft(x, nfft);
    P2 = abs(Y / N);
    amp = P2(1:(nfft/2 + 1));
    amp(2:end-1) = 2 * amp(2:end-1);
    f = fs * (0:(nfft/2)) / nfft;
end

function [ts, ynorm, tau63_ms] = extract_norm_step(t, y, t_step, win_s)
    y0 = mean(y((t >= t_step - 0.25) & (t < t_step)));
    y1 = mean(y((t >= t_step + 0.80) & (t <= t_step + 1.20)));
    m  = (t >= t_step) & (t <= t_step + win_s);
    ts = t(m) - t_step;
    ynorm = (y(m) - y0) / (y1 - y0);
    idx = find(ynorm >= 0.632, 1, 'first');
    if isempty(idx)
        tau63_ms = NaN;
    else
        tau63_ms = ts(idx) * 1000;
    end
end

function [v_out, u_out] = sim_cl_2dof_pi(vref, Kdc, tau_m, udz_plant, Kp, Ki, Kff, udz_ctrl, eband, Ts, delay_samp, do_sat)
    N = numel(vref);
    v_out = zeros(N, 1);
    u_out = zeros(N, 1);
    v_out(1) = vref(1);
    a = exp(-Ts / tau_m);
    b = Kdc * (1.0 - a);
    I_acc = 0.0; prev_e = 0.0; in_band = false;
    u_buf = zeros(N + delay_samp, 1);
    u_init = Kff * vref(1) + udz_ctrl;
    u_buf(1:(delay_samp + 1)) = u_init;
    u_out(1) = u_init;
    for k = 2:N
        u_eff = max(0.0, u_buf(k-1) - udz_plant);
        v_out(k) = a * v_out(k-1) + b * u_eff;
        e = vref(k) - v_out(k);
        u_ff = Kff * vref(k) + udz_ctrl;
        u_p  = Kp * e;
        wb   = abs(e) < eband;
        i_cand = I_acc;
        if wb
            ep = prev_e;
            if ~in_band, ep = e; end
            i_cand = I_acc + 0.5 * Ki * Ts * (e + ep);
        end
        u_unsat = u_ff + u_p + i_cand;
        if do_sat
            if u_unsat > 100.0
                u_cmd = 100.0;
                if e < 0 && wb, I_acc = i_cand; end
            elseif u_unsat < 0.0
                u_cmd = 0.0;
                if e > 0 && wb, I_acc = i_cand; end
            else
                u_cmd = u_unsat;
                if wb, I_acc = i_cand; end
            end
        else
            u_cmd = u_unsat;
            if wb, I_acc = i_cand; end
        end
        u_out(k) = u_cmd;
        u_buf(k + delay_samp) = u_cmd;
        prev_e = e; in_band = wb;
    end
end

function r = corr_manual(x, y)
    xm = x - mean(x);
    ym = y - mean(y);
    r = sum(xm .* ym) / sqrt(sum(xm.^2) * sum(ym.^2));
end

function draw_sum_x(ax, cx, cy, r, col)
    th = linspace(0, 2*pi, 60);
    patch(ax, cx + r*cos(th), cy + r*sin(th), 'w', 'EdgeColor', col, 'LineWidth', 1.15);
    d = r * cos(pi/4);
    plot(ax, [cx - d, cx + d], [cy - d, cy + d], '-', 'Color', col, 'LineWidth', 0.9);
    plot(ax, [cx - d, cx + d], [cy + d, cy - d], '-', 'Color', col, 'LineWidth', 0.9);
end