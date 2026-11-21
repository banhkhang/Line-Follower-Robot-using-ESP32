%% sysid_motor_wifi.m
% Thu thập dữ liệu RPM, PWM, seq qua Wi-Fi (HTTP binary dump) từ ESP32
% và ước lượng hàm truyền bậc 1: G(s) = RPM(s)/PWM(s) = Kdc / (tau_m*s + 1)

clear; clc; close all;

%% 1. Cấu hình
BASE_URL = 'http://192.168.4.1:8080';
MODE     = 'id30';   % 'check5' (5s kiểm tra phần cứng) hoặc 'id30' (30s nhận dạng)
MOTOR    = 'AB';      % 'A', 'B', hoặc 'AB'
TS       = 0.005;    % Chu kỳ lấy mẫu cứng trên ESP32 (200 Hz)

opts_json = weboptions('ContentType', 'json', 'Timeout', 10);
opts_bin  = weboptions('ContentType', 'binary', 'Timeout', 20);

%% 2. Gửi lệnh Start & chờ ESP32 chạy xong trên gá đỡ
fprintf('Gửi lệnh bắt đầu (%s, Motor %s)...\n', MODE, MOTOR);
start_url = sprintf('%s/start?mode=%s&motor=%s', BASE_URL, MODE, MOTOR);
res = webread(start_url, opts_json);
disp(res);

t0 = tic;
while true
    pause(0.5);
    st = webread([BASE_URL '/status'], opts_json);
    fprintf('\r[%.1fs] Trạng thái: %-8s | Đã ghi: %4d / %4d mẫu', ...
            toc(t0), st.state, st.logged, st.total);
    if strcmp(st.state, 'done')
        fprintf('\n');
        break;
    end
end

%% 3. Tải file nhị phân /dump.bin và giải mã cấu trúc 12-byte
fprintf('Đang tải dump.bin...\n');
raw = webread([BASE_URL '/dump.bin'], opts_bin);
n_bytes = numel(raw);
assert(mod(n_bytes, 12) == 0, 'Kích thước dữ liệu không chia hết cho 12 bytes!');

M = reshape(raw, 12, []);
N = size(M, 2);

seq       = double(typecast(reshape(M(1:2,  :), [], 1), 'uint16'));
pwm_pct   = double(typecast(reshape(M(3:4,  :), [], 1), 'int16')) / 100.0;
rpm_a     = double(typecast(reshape(M(5:6,  :), [], 1), 'int16')) / 10.0;
rpm_b     = double(typecast(reshape(M(7:8,  :), [], 1), 'int16')) / 10.0;
dt_us_err = double(typecast(reshape(M(9:10, :), [], 1), 'uint16'));
enc_delta = double(typecast(reshape(M(11:12,:), [], 1), 'int16'));

% Tái dựng trục thời gian hoàn toàn từ seq của ESP32
t = seq * TS;

% Chọn kênh RPM mục tiêu
if strcmpi(MOTOR, 'B')
    rpm = rpm_b;
else
    rpm = rpm_a;
end

%% 4. Kiểm tra toàn vẹn dữ liệu & Jitter Core 1
seq_ok = all(diff(seq) == 1);
fprintf('Số mẫu: %d | Liên tục (diff(seq)==1): %d | Max jitter: %d us | Mean jitter: %.1f us\n', ...
        N, seq_ok, max(dt_us_err), mean(dt_us_err));
if ~seq_ok
    warning('Phát hiện mất mẫu hoặc nhảy seq! Không nên dùng dữ liệu này cho System ID.');
end

%% 5. Lưu file CSV & MAT
stamp = datestr(now, 'yyyymmdd_HHMMSS');
fname = sprintf('sysid_motor%s_%s_%s', MOTOR, MODE, stamp);
T = table(seq, t, pwm_pct, rpm_a, rpm_b, dt_us_err, enc_delta);
writetable(T, [fname '.csv']);
save([fname '.mat'], 'seq', 't', 'pwm_pct', 'rpm_a', 'rpm_b', 'dt_us_err', 'enc_delta', 'TS');
fprintf('Đã lưu: %s.csv và %s.mat\n', fname, fname);

%% 6. Vẽ đồ thị kiểm tra
figure('Name', ['Motor ' MOTOR ' Raw Log'], 'Position', [100 100 1000 650]);
subplot(3,1,1);
stairs(t, pwm_pct, 'LineWidth', 1.2); grid on;
ylabel('PWM [%]'); title(sprintf('Motor %s (%s) — Dữ liệu thực nghiệm trên gá đỡ', MOTOR, MODE));

subplot(3,1,2);
plot(t, rpm_a, 'b', 'DisplayName', 'RPM A'); hold on;
plot(t, rpm_b, 'r--', 'DisplayName', 'RPM B'); grid on;
ylabel('Tốc độ [RPM]'); legend('Location', 'best');

subplot(3,1,3);
plot(t, dt_us_err, 'k'); grid on;
ylabel('Jitter [us]'); xlabel('Thời gian t = seq \times 0.005 [s]');

%% 7. Nhận dạng hàm truyền (chỉ chạy cho chế độ id30)
if strcmp(MODE, 'id30') && license('test', 'Identification_Toolbox')
    % Tách 2/3 Estimation (0..20s) và 1/3 Validation (20..30s)
    idx_est = (t >= 1.0) & (t < 20.0);
    idx_val = (t >= 20.0) & (t <= 29.0);

    z_est = iddata(rpm(idx_est), pwm_pct(idx_est), TS);
    z_val = iddata(rpm(idx_val), pwm_pct(idx_val), TS);

    % Ước lượng mô hình bậc 1 (G1) và bậc 2 (G2) để kiểm tra cực phụ
    G1 = tfest(z_est, 1, 0);
    G2 = tfest(z_est, 2, 0);

    % Trích xuất Kdc [RPM/%PWM] và hằng số thời gian tau_m [s]
    Kdc   = dcgain(G1);
    p1    = pole(G1);
    tau_m = -1 / real(p1(1));

    fprintf('\n=== KẾT QUẢ NHẬN DẠNG BẬC 1: G1(s) = Kdc / (tau_m*s + 1) ===\n');
    fprintf('Kdc   = %.4f RPM / %%PWM\n', Kdc);
    fprintf('tau_m = %.4f s (%.1f ms)\n', tau_m, tau_m * 1000);
    fprintf('Cực G2: %.2f, %.2f rad/s (tau2 = %.4f s)\n', ...
            real(pole(G2)), -1/min(real(pole(G2))));

    figure('Name', ['System ID Validation - Motor ' MOTOR]);
    compare(z_val, G1, G2);
    grid on;
    title(sprintf('Validation Motor %s | Kdc = %.3f RPM/%%, \\tau_m = %.3f s', MOTOR, Kdc, tau_m));
end