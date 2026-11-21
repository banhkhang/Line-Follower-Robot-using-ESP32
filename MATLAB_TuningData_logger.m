%% verify_2dof_pi_wifi.m
% Nghiệm thu vòng kín 2-DOF PI 10 giây (2000 mẫu @ 200 Hz) qua Wi-Fi
% Đánh giá tự động Ts,5% (<= 0.09 s), OS% (< 5%), ess và PWMmax

clear; clc; close all;

%% 1. Cấu hình chạy thử
BASE_URL = 'http://192.168.4.1:8080';
MOTOR    = 'AB';         % 'A', 'B', hoặc 'AB' (chạy đồng thời 2 bánh)
PRESET   = 'floor_13kg';      % 'bench' (trên gá đỡ) hoặc 'floor_13kg' (sàn thật 1.3 kg)
TS       = 0.005;

% Tiêu chuẩn nghiệm thu
TS_LIMIT_S = 0.090;      % Ts,5% <= 0.09 s
OS_LIMIT_P = 5.0;        % OS% < 5%

opts_json = weboptions('ContentType', 'json', 'Timeout', 10);
opts_bin  = weboptions('ContentType', 'binary', 'Timeout', 20);

%% 2. Gửi tham số 2-DOF PI nóng qua /tune
if strcmp(PRESET, 'floor_13kg')
    pA = struct('kp', 310.0, 'ki', 2200.0, 'kff', 86.90, 'udz', 4.85, 'eband', 0.020);
    pB = struct('kp', 355.0, 'ki', 2100.0, 'kff', 81.80, 'udz', 5.45, 'eband', 0.020);
else
    pA = struct('kp', 227.0, 'ki', 2978.0, 'kff', 85.72, 'udz', 2.62, 'eband', 0.030);
    pB = struct('kp', 272.0, 'ki', 2856.0, 'kff', 82.22, 'udz', 1.66, 'eband', 0.030);
end

fprintf('Nạp tham số preset [%s]...\n', PRESET);
rA = webread(sprintf('%s/tune?motor=A&kp=%.2f&ki=%.2f&kff=%.2f&udz=%.2f&eband=%.4f', ...
             BASE_URL, pA.kp, pA.ki, pA.kff, pA.udz, pA.eband), opts_json);
rB = webread(sprintf('%s/tune?motor=B&kp=%.2f&ki=%.2f&kff=%.2f&udz=%.2f&eband=%.4f', ...
             BASE_URL, pB.kp, pB.ki, pB.kff, pB.udz, pB.eband), opts_json);
fprintf('  Motor A: Kp=%.2f, Ki=%.2f, Kff=%.2f, udz=%.2f\n', rA.kp, rA.ki, rA.kff, rA.udz);
fprintf('  Motor B: Kp=%.2f, Ki=%.2f, Kff=%.2f, udz=%.2f\n', rB.kp, rB.ki, rB.kff, rB.udz);

%% 3. Chạy bài nghiệm thu 10 giây
fprintf('Bắt đầu chạy vòng kín 10s (Motor %s)...\n', MOTOR);
webread(sprintf('%s/start?mode=cl10&motor=%s', BASE_URL, MOTOR), opts_json);

t0 = tic;
while true
    pause(0.4);
    st = webread([BASE_URL '/status'], opts_json);
    fprintf('\r[%.1fs] Trạng thái: %-8s | Đã ghi: %4d / %4d mẫu', ...
            toc(t0), st.state, st.logged, st.total);
    if strcmp(st.state, 'done')
        fprintf('\n');
        break;
    end
end

%% 4. Tải dump.bin (14 bytes/mẫu) & kiểm tra toàn vẹn
raw = webread([BASE_URL '/dump.bin'], opts_bin);
assert(mod(numel(raw), 14) == 0, 'Kích thước dump.bin không chia hết cho 14 bytes!');

M = reshape(raw, 14, []);
N = size(M, 2);

seq       = double(typecast(reshape(M(1:2,  :), [], 1), 'uint16'));
vref      = double(typecast(reshape(M(3:4,  :), [], 1), 'int16')) / 1000.0;
va        = double(typecast(reshape(M(5:6,  :), [], 1), 'int16')) / 1000.0;
vb        = double(typecast(reshape(M(7:8,  :), [], 1), 'int16')) / 1000.0;
pwm_a     = double(typecast(reshape(M(9:10, :), [], 1), 'int16')) / 100.0;
pwm_b     = double(typecast(reshape(M(11:12,:), [], 1), 'int16')) / 100.0;
dt_us_err = double(typecast(reshape(M(13:14,:), [], 1), 'uint16'));
t         = seq * TS;

seq_ok = all(diff(seq) == 1);
fprintf('Mẫu: %d | Liên tục: %d | Max jitter: %d us | Mean jitter: %.1f us\n', ...
        N, seq_ok, max(dt_us_err), mean(dt_us_err));

%% 5. Lưu file CSV & MAT
stamp = datestr(now, 'yyyymmdd_HHMMSS');
fname = sprintf('cl10_%s_%s_%s', PRESET, MOTOR, stamp);
Tlog  = table(seq, t, vref, va, vb, pwm_a, pwm_b, dt_us_err);
writetable(Tlog, [fname '.csv']);
save([fname '.mat'], 'seq', 't', 'vref', 'va', 'vb', 'pwm_a', 'pwm_b', 'dt_us_err', 'pA', 'pB', 'TS');
fprintf('Đã lưu: %s.csv và %s.mat\n', fname, fname);

%% 6. Tự động đánh giá 5 bước nhảy nghiệm thu
steps = {
    '0.5 -> 0.7 (1)', 2.00, 3.30, 0.5, 0.7;
    '0.7 -> 0.5',     3.30, 4.60, 0.7, 0.5;
    '0.5 -> 0.3',     4.60, 5.90, 0.5, 0.3;
    '0.3 -> 0.5',     5.90, 7.20, 0.3, 0.5;
    '0.5 -> 0.7 (2)', 7.20, 8.50, 0.5, 0.7;
};

channels = {};
if contains(upper(MOTOR), 'A'), channels{end+1} = {'Motor A', va, pwm_a}; end
if contains(upper(MOTOR), 'B'), channels{end+1} = {'Motor B', vb, pwm_b}; end

for c = 1:numel(channels)
    ch_name = channels{c}{1};
    v_meas  = channels{c}{2};
    u_pwm   = channels{c}{3};

    fprintf('\n==================== NGHIỆM THU %s (%s) ====================\n', ch_name, PRESET);
    fprintf('%-15s | %-9s | %-8s | %-11s | %-9s | %s\n', ...
            'Bước nhảy', 'Ts,5% (s)', 'OS% (%)', 'ess (mm/s)', 'PWMmax(%)', 'Kết quả');
    fprintf('%s\n', repmat('-', 1, 74));

    for k = 1:size(steps, 1)
        s_name = steps{k,1};
        t_s = steps{k,2}; t_e = steps{k,3};
        v0  = steps{k,4}; v1  = steps{k,5};
        dv  = v1 - v0;

        idx = (t >= t_s) & (t < t_e);
        ts  = t(idx) - t_s;
        ym  = v_meas(idx);
        um  = u_pwm(idx);

        band    = 0.05 * abs(dv);
        outside = find(abs(ym - v1) > band);
        if isempty(outside)
            Ts5 = 0.0;
        elseif outside(end) >= numel(ts)
            Ts5 = NaN;
        else
            Ts5 = ts(outside(end) + 1);
        end

        

        if dv > 0
            os_pct = max(0, (max(ym) - v1) / abs(dv) * 100);
        else
            os_pct = max(0, (v1 - min(ym)) / abs(dv) * 100);
        end

        ess_mm  = mean(v1 - ym(ts >= (t_e - t_s - 0.30))) * 1000.0;
        pwm_max = max(um);

        pass = (~isnan(Ts5)) && (Ts5 <= TS_LIMIT_S) && (os_pct < OS_LIMIT_P);
        verdict = 'ĐẠT'; if ~pass, verdict = 'CHƯA ĐẠT'; end

        fprintf('%-15s | %9.4f | %8.2f | %+11.2f | %9.1f | %s\n', ...
                s_name, Ts5, os_pct, ess_mm, pwm_max, verdict);
    end
end

%% 7. Vẽ đồ thị nghiệm thu
figure('Name', ['2-DOF PI Verification - ' PRESET], 'Position', [80 60 1100 720]);

subplot(3,1,1);
plot(t, vref, 'k--', 'LineWidth', 1.4, 'DisplayName', 'v_{ref}'); hold on;
if contains(upper(MOTOR), 'A'), plot(t, va, 'b', 'LineWidth', 1.1, 'DisplayName', 'v_A'); end
if contains(upper(MOTOR), 'B'), plot(t, vb, 'r', 'LineWidth', 1.1, 'DisplayName', 'v_B'); end
grid on; ylabel('Vận tốc [m/s]'); legend('Location', 'best');
title(sprintf('Đáp ứng vòng kín 2-DOF PI 10s (%s, Motor %s)', PRESET, MOTOR));

subplot(3,1,2);
if contains(upper(MOTOR), 'A'), plot(t, pwm_a, 'b', 'DisplayName', 'PWM A'); hold on; end
if contains(upper(MOTOR), 'B'), plot(t, pwm_b, 'r', 'DisplayName', 'PWM B'); end
grid on; ylabel('PWM [%]'); ylim([-5 105]); legend('Location', 'best');

subplot(3,1,3);
if strcmpi(MOTOR, 'AB')
    plot(t, (va - vb) * 1000, 'm', 'LineWidth', 1.0);
    ylabel('Lệch v_A - v_B [mm/s]');
else
    plot(t, dt_us_err, 'k');
    ylabel('Jitter [us]');
end
grid on; xlabel('Thời gian t = seq \times 0.005 [s]');