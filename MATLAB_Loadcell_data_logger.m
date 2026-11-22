%% HX711 Multi-Point Wi-Fi Calibration Script
clear; clc; close all;

BASE_URL       = 'http://192.168.4.1:8080';
SAMPLES_PER_PT = 60;      % 60 samples @ 10 SPS = 6.0 s per weight
SETTLE_SEC     = 1.5;     % Extra wait after pressing Enter for PETG tray damping
FULL_SCALE_G   = 2000;    % Rated range for %FS error metrics
MAD_SIGMA_LIM  = 3.5;     % Outlier rejection threshold

jsonOpts = weboptions('Timeout', 10, 'ContentType', 'json');
binOpts  = weboptions('Timeout', 10, 'ContentType', 'binary');

%% 1. Verify Connection & HX711 Health
fprintf('Connecting to %s ...\n', BASE_URL);
st = webread([BASE_URL '/status'], jsonOpts);
if ~st.valid
    error('HX711 not reporting valid samples (timeouts=%d). Check DOUT/SCK/XI.', st.timeouts);
end
fprintf('Connected. Measured rate: %.2f SPS | Timeouts: %d | Saturated: %d\n\n', ...
    st.sps, st.timeouts, st.sat_count);

%% 2. Interactive Multi-Point Capture (Loading + Unloading)
fprintf('Enter known mass in grams for each point (e.g., 0 -> 500 -> 1000 -> 2000 -> 1000 -> 500 -> 0).\n');
fprintf('Press ENTER on an empty line when finished.\n\n');

pts = struct('mass_g', {}, 't_s', {}, 'raw', {}, 'clean', {}, ...
             'n_outliers', {}, 'mean_clean', {}, 'std_clean', {}, ...
             'drift_cps', {}, 'phase', {});
prev_mass = -inf;
k = 0;

while true
    str = input(sprintf('Point #%d - Known mass [g] (empty to finish): ', k + 1), 's');
    if isempty(strtrim(str))
        if k < 2
            fprintf('Need at least 2 points.\n');
            continue;
        end
        break;
    end
    mass_g = str2double(str);
    if isnan(mass_g) || mass_g < 0
        fprintf('Invalid mass.\n');
        continue;
    end

    fprintf('  Settling %.1f s...\n', SETTLE_SEC);
    pause(SETTLE_SEC);

    % Trigger capture on ESP32 Core 1
    capResp = webread(sprintf('%s/capture?n=%d', BASE_URL, SAMPLES_PER_PT), jsonOpts);
    fprintf('  Capturing %d samples (~%.1f s)...', capResp.target, capResp.est_sec);

    while true
        pause(0.4);
        st = webread([BASE_URL '/status'], jsonOpts);
        if strcmp(st.state, 'done')
            break;
        end
    end
    fprintf(' done.\n');

    % Download and unpack binary dump
    bytes = webread([BASE_URL '/dump.bin'], binOpts);
    hdr   = typecast(bytes(1:16), 'uint32');
    if hdr(1) ~= uint32(hex2dec('31313758'))
        error('Bad binary magic: 0x%08X', hdr(1));
    end
    n = double(hdr(3));
    payload = reshape(bytes(17 : 16 + n * 8), 8, n);
    t_ms    = double(typecast(reshape(payload(1:4, :), [], 1), 'uint32'));
    raw     = double(typecast(reshape(payload(5:8, :), [], 1), 'int32'));
    t_s     = (t_ms - t_ms(1)) / 1000.0;

    % Robust MAD outlier filter
    med    = median(raw);
    madVal = 1.4826 * median(abs(raw - med));
    thresh = max(MAD_SIGMA_LIM * madVal, 32);
    inlier = abs(raw - med) <= thresh;

    clean = raw(inlier);
    t_in  = t_s(inlier);

    % Intra-capture linear drift (counts/sec)
    p_drift = polyfit(t_in, clean, 1);

    k = k + 1;
    pts(k).mass_g     = mass_g;
    pts(k).t_s        = t_s;
    pts(k).raw        = raw;
    pts(k).clean      = clean;
    pts(k).n_outliers = sum(~inlier);
    pts(k).mean_clean = mean(clean);
    pts(k).std_clean  = std(clean);
    pts(k).drift_cps  = p_drift(1);

    if k == 1 || mass_g >= prev_mass
        pts(k).phase = 'load';
    else
        pts(k).phase = 'unload';
    end
    prev_mass = mass_g;

    fprintf('  -> Mean: %10.2f | Std: %6.2f cnt | Outliers: %d/%d | Drift: %+.2f cnt/s\n\n', ...
        pts(k).mean_clean, pts(k).std_clean, pts(k).n_outliers, n, pts(k).drift_cps);
end

%% 3. Linear Fit & Diagnostics
masses     = [pts.mass_g]';
means      = [pts.mean_clean]';
stds_cnt   = [pts.std_clean]';
drifts_cps = [pts.drift_cps]';

% Fit: raw = slope * mass_g + tare_offset
p = polyfit(masses, means, 1);
counts_per_g = p(1);
tare_counts  = p(2);
g_per_count  = 1.0 / counts_per_g;

pred_mass_g = (means - tare_counts) * g_per_count;
resid_g     = pred_mass_g - masses;

ss_res = sum((means - polyval(p, masses)).^2);
ss_tot = sum((means - mean(means)).^2);
R2     = 1 - ss_res / ss_tot;

rmse_g     = sqrt(mean(resid_g.^2));
max_err_g  = max(abs(resid_g));
max_err_fs = 100 * max_err_g / FULL_SCALE_G;
stds_g     = stds_cnt * abs(g_per_count);
drifts_gps = drifts_cps * abs(g_per_count);

% Hysteresis analysis (matching masses across load/unload)
unique_m = unique(masses);
hyst_m   = [];
hyst_g   = [];
for i = 1:numel(unique_m)
    idx_u = find(masses == unique_m(i));
    if numel(idx_u) >= 2
        % Difference between unloading and loading predictions at same mass
        hyst_m(end+1, 1) = unique_m(i); %#ok<SAGROW>
        hyst_g(end+1, 1) = pred_mass_g(idx_u(end)) - pred_mass_g(idx_u(1)); %#ok<SAGROW>
    end
end

zero_idx = find(masses == 0);
zero_drift_g = NaN;
if numel(zero_idx) >= 2
    zero_drift_g = (means(zero_idx(end)) - means(zero_idx(1))) * g_per_count;
end

%% 4. Console Report & C Header Output
fprintf('================ CALIBRATION REPORT ================\n');
fprintf('Slope (counts/g)    : %.6f\n', counts_per_g);
fprintf('Scale (g/count)     : %.8f\n', g_per_count);
fprintf('Tare Offset (counts): %.2f\n', tare_counts);
fprintf('Linearity R^2       : %.8f\n', R2);
fprintf('RMSE                : %.3f g\n', rmse_g);
fprintf('Max Residual Error  : %.3f g (%.3f %%FS)\n', max_err_g, max_err_fs);
fprintf('Mean 1-Sigma Noise  : %.3f g (%.1f mg)\n', mean(stds_g), mean(stds_g)*1000);
fprintf('Max Intra-Cap Drift : %.4f g/s\n', max(abs(drifts_gps)));
if ~isnan(zero_drift_g)
    fprintf('Zero-Return Drift   : %+.3f g\n', zero_drift_g);
end
if ~isempty(hyst_g)
    fprintf('Max Hysteresis      : %.3f g (%.3f %%FS)\n', max(abs(hyst_g)), 100*max(abs(hyst_g))/FULL_SCALE_G);
end
fprintf('====================================================\n\n');

fprintf('/* Paste into components/application/config.h */\n');
fprintf('#define HX711_CAL_TARE_COUNTS   (%.3ff)\n', tare_counts);
fprintf('#define HX711_CAL_COUNTS_PER_G  (%.6ff)\n', counts_per_g);
fprintf('/* Runtime formula: float mass_g = (filtered_raw - HX711_CAL_TARE_COUNTS) / HX711_CAL_COUNTS_PER_G; */\n\n');

%% 5. Save Raw Dataset
fname = sprintf('hx711_cal_%s.mat', datestr(now, 'yyyymmdd_HHMMSS'));
save(fname, 'pts', 'counts_per_g', 'tare_counts', 'g_per_count', 'R2', 'rmse_g', 'resid_g', 'hyst_m', 'hyst_g');
fprintf('Saved dataset to %s\n', fname);

%% 6. Diagnostic Plots
is_load = strcmp({pts.phase}, 'load');

figure('Name', 'HX711 Calibration Diagnostics', 'Color', 'w', 'Position', [100 100 1100 720]);

% (1) Linear Fit
subplot(2, 2, 1); hold on; grid on;
m_line = linspace(0, max(masses) * 1.05, 100);
plot(m_line, polyval(p, m_line), 'k-', 'LineWidth', 1.2, 'DisplayName', 'Linear Fit');
plot(masses(is_load),  means(is_load),  'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Loading');
plot(masses(~is_load), means(~is_load), 'rs', 'MarkerFaceColor', 'r', 'DisplayName', 'Unloading');
xlabel('Known Mass (g)'); ylabel('HX711 Raw Counts');
title(sprintf('Calibration Curve (R^2 = %.6f)', R2));
legend('Location', 'best');

% (2) Residuals & Hysteresis
subplot(2, 2, 2); hold on; grid on;
yline(0, 'k--');
plot(masses(is_load),  resid_g(is_load),  'bo-', 'MarkerFaceColor', 'b', 'DisplayName', 'Loading');
if any(~is_load)
    plot(masses(~is_load), resid_g(~is_load), 'rs-', 'MarkerFaceColor', 'r', 'DisplayName', 'Unloading');
end
xlabel('Known Mass (g)'); ylabel('Residual Error (g)');
title(sprintf('Linearity & Hysteresis (RMSE = %.2f g)', rmse_g));
legend('Location', 'best');

% (3) Per-Point Noise Std
subplot(2, 2, 3); grid on;
bar(1:k, stds_g);
xticks(1:k);
xticklabels(arrayfun(@(x) sprintf('%g g', x), masses, 'UniformOutput', false));
xtickangle(45);
ylabel('1-\sigma Noise (g)');
title('Noise Stability per Capture Point');

% (4) Normalized Raw Traces (drift & outlier visibility)
subplot(2, 2, 4); hold on; grid on;
for i = 1:k
    delta_g = (pts(i).raw - pts(i).mean_clean) * g_per_count;
    plot(pts(i).t_s, delta_g, 'DisplayName', sprintf('#%d (%g g)', i, pts(i).mass_g));
end
xlabel('Capture Time (s)'); ylabel('Deviation from Mean (g)');
title('Raw Time-Series (Drift & Spike Check)');
legend('Location', 'bestoutside');