%% Live 7-Sensor vs 5-Sensor Characterization & Edge Linearization
clear; close all;

COM_PORT  = "COM24";      % <-- Change to your ESP32 USB UART port
BAUD_RATE = 115200;
PITCH_MM  = 9.0;         % Physical sensor spacing (mm)
DIVERGE_THRESH_MM = 0.25;% Threshold (mm) where err_5 diverges from err_7

% Physical coordinates (mm)
pos7 = (3:-1:-3) * PITCH_MM;   % [-27, -18, -9, 0, +9, +18, +27]
pos5 = (2:-1:-2) * PITCH_MM;   % [     -18, -9, 0, +9, +18     ]

% Storage arrays
norm7_log = [];   % Nx7 normalized values (S1..S7)
err7_esp  = [];   % Nx1 error printed by ESP32
err7_mat  = [];   % Nx1 error recomputed in MATLAB from S1..S7
err5_mat  = [];   % Nx1 error computed in MATLAB from S2..S6 only
t_sec     = [];

% Open serial port
s = serialport(COM_PORT, BAUD_RATE);
configureTerminator(s, "LF");
flush(s);

% Live preview figure (closing this window stops recording cleanly)
fig = figure('Name', 'Live 7-SS vs 5-SS Capture (Close window to finish)', ...
             'NumberTitle', 'off', 'Color', 'w');
h7 = animatedline('Color', [0 0.45 0.74], 'LineWidth', 1.8, 'DisplayName', '7-Sensor Ref (S1-S7)');
h5 = animatedline('Color', [0.85 0.33 0.10], 'LineWidth', 1.8, 'LineStyle', '--', 'DisplayName', '5-Sensor (S2-S6)');
grid on; xlabel('Sample #'); ylabel('Line Position Error (mm)');
title('Slide array slowly across line (-20 mm to +20 mm) — Close window when done');
legend('Location', 'northwest');
ylim([-22 22]);

fprintf('Connected to %s. Wave/slide sensors across the line...\n', COM_PORT);
fprintf('Close the live plot window when finished collecting.\n\n');

t0 = tic;
pattern = 'LINE \(S1-S7\) \[[^\]]+\]\s*\|\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*\|\s*err=\s*([+-]?\d+\.\d+)\s*mm\s*\|\s*OK';

while isvalid(fig)
    if s.NumBytesAvailable > 0
        lineStr = readline(s);
        tokens = regexp(char(lineStr), pattern, 'tokens', 'once');
        if ~isempty(tokens)
            vals = str2double(tokens(1:7));   % [S1 S2 S3 S4 S5 S6 S7]
            e_esp = str2double(tokens{8});

            % Recompute 7-sensor & 5-sensor weighted averages identically to C code
            w7 = sum(vals);
            w5 = sum(vals(2:6));

            if w7 >= 25.0 && w5 >= 25.0
                e7 = sum(vals .* pos7) / w7;
                e5 = sum(vals(2:6) .* pos5) / w5;

                norm7_log(end+1, :) = vals;
                err7_esp(end+1, 1)  = e_esp;
                err7_mat(end+1, 1)  = e7;
                err5_mat(end+1, 1)  = e5;
                t_sec(end+1, 1)     = toc(t0);

                idx = numel(err7_mat);
                addpoints(h7, idx, e7);
                addpoints(h5, idx, e5);
                if mod(idx, 5) == 0
                    drawnow limitrate;
                end
            end
        end
    else
        pause(0.005);
    end
end

clear s; % Release COM port
fprintf('Captured %d valid samples.\n', numel(err7_mat));
if numel(err7_mat) < 10
    error('Not enough samples captured. Check COM port and USE_TELEMETRY=0.');
end

%% 1. Analyze Divergence Using Relative Percentage Tolerance (5%, 7%, or 10%)
REL_TOL_PCT = 7.0;          % <-- Change to 5.0, 7.0, or 10.0 (%)
CENTER_GUARD_MM = 4.0;      % Ignore center region where |err_7| is near 0

[e7_sorted, sortIdx] = sort(err7_mat);
e5_sorted   = err5_mat(sortIdx);
diff_sorted = abs(e7_sorted - e5_sorted);
n7_sorted   = norm7_log(sortIdx, :);

% Compute relative percentage error (%) outside the center guard zone
rel_err_pct = zeros(size(e7_sorted));
validPctMask = abs(e7_sorted) >= CENTER_GUARD_MM;
rel_err_pct(validPctMask) = (diff_sorted(validPctMask) ./ abs(e7_sorted(validPctMask))) * 100.0;

% Find where relative error first exceeds REL_TOL_PCT (5%, 7%, or 10%)
inaccurateMask = validPctMask & (rel_err_pct > REL_TOL_PCT);
if any(inaccurateMask)
    e7_diverge_start = min(abs(e7_sorted(inaccurateMask)));
    e5_diverge_start = min(abs(e5_sorted(inaccurateMask)));
else
    e7_diverge_start = 9.0;
    e5_diverge_start = 9.0;
end

% Outer sensor onset (ignore small <15% S7 floor noise; look for real line edge >= 20%)
outer_active = (n7_sorted(:,1) >= 20) | (n7_sorted(:,7) >= 20);
e7_outer_onset = min(abs(e7_sorted(outer_active)));
if isempty(e7_outer_onset), e7_outer_onset = NaN; end

max_reach_7 = max(abs(e7_sorted));
max_reach_5 = max(abs(e5_sorted));

%% 2. Fit Piecewise Linear Edge Correction Only Beyond e5_diverge_start
edgeMask = (abs(e5_sorted) >= e5_diverge_start) & (abs(e7_sorted) <= 16.5);
if nnz(edgeMask) >= 3
    x_edge = abs(e5_sorted(edgeMask)) - e5_diverge_start;
    y_edge = abs(e7_sorted(edgeMask)) - e5_diverge_start;
    K_slope = (x_edge' * y_edge) / (x_edge' * x_edge);
else
    K_slope = 1.0;
end

err5_corr = e5_sorted;
corrIdx = abs(err5_corr) > e5_diverge_start;
err5_corr(corrIdx) = sign(err5_corr(corrIdx)) .* ...
    (e5_diverge_start + K_slope .* (abs(err5_corr(corrIdx)) - e5_diverge_start));

%% 3. Print Summary & C Code Constants
fprintf('\n==================== ANALYSIS RESULTS ====================\n');
fprintf('Outer sensors (S1/S7) start seeing line at : |err_7| = %.2f mm\n', e7_outer_onset);
fprintf('5-SS diverges (>%.2f mm) from 7-SS at      : |err_7| = %.2f mm (|err_5| = %.2f mm)\n', ...
        DIVERGE_THRESH_MM, e7_diverge_start, e5_diverge_start);
fprintf('Max measured reach (7-SS vs 5-SS)          : %.2f mm vs %.2f mm\n', max_reach_7, max_reach_5);
fprintf('Fitted edge correction gain (up to 16 mm)  : K_EDGE_GAIN = %.4f\n', K_slope);
fprintf('==========================================================\n\n');

fprintf('--- Copy into sensor_service.c (inside #if USE_TELEMETRY) ---\n');
fprintf('#define EDGE_CORR_START_MM   %.2ff\n', e5_diverge_start);
fprintf('#define EDGE_CORR_GAIN       %.4ff\n', K_slope);

%% 4. Diagnostic Plots
figure('Name', '7-Sensor vs 5-Sensor Calibration Curve', 'Color', 'w', 'Position', [100 100 1000 700]);

subplot(2,2,1);
plot(t_sec, err7_mat, 'b-', 'LineWidth', 1.5); hold on;
plot(t_sec, err5_mat, 'r--', 'LineWidth', 1.5);
yline(16, 'k:', 'Target +-16 mm'); yline(-16, 'k:');
grid on; xlabel('Time (s)'); ylabel('Error (mm)');
title('Time Series: 7-SS Reference vs 5-SS');
legend('7-SS (S1-S7)', '5-SS (S2-S6)', 'Location', 'best');

subplot(2,2,2);
plot(e7_sorted, e7_sorted, 'k-', 'LineWidth', 1.2, 'DisplayName', 'Ideal 1:1 (7-SS)'); hold on;
plot(e7_sorted, e5_sorted, 'r.', 'MarkerSize', 8, 'DisplayName', 'Raw 5-SS');
plot(e7_sorted, err5_corr, 'g.', 'MarkerSize', 8, 'DisplayName', 'Corrected 5-SS');
xline(e7_diverge_start, 'm--', 'Divergence'); xline(-e7_diverge_start, 'm--');
grid on; xlabel('True Reference Error err_7 (mm)'); ylabel('Measured Error (mm)');
title('Transfer Curve: Raw vs Linearized 5-SS');
legend('Location', 'northwest');

subplot(2,2,3);
plot(e7_sorted, e7_sorted - e5_sorted, 'r.', 'DisplayName', 'Raw Error (err_7 - err_5)'); hold on;
plot(e7_sorted, e7_sorted - err5_corr, 'g.', 'DisplayName', 'After Correction');
yline(DIVERGE_THRESH_MM, 'k--'); yline(-DIVERGE_THRESH_MM, 'k--');
grid on; xlabel('True Reference Error err_7 (mm)'); ylabel('Residual Error (mm)');
title('5-SS Inaccuracy Across Track Width');
legend('Location', 'best');

subplot(2,2,4);
plot(e7_sorted, n7_sorted(:,1), 'LineWidth', 1.3, 'DisplayName', 'S1 (-27 mm)'); hold on;
plot(e7_sorted, n7_sorted(:,7), 'LineWidth', 1.3, 'DisplayName', 'S7 (+27 mm)');
xline(e7_outer_onset, 'k--', 'S1/S7 Onset'); xline(-e7_outer_onset, 'k--');
grid on; xlabel('True Reference Error err_7 (mm)'); ylabel('Normalized Signal (%)');
title('Outer Sensors S1 & S7 Activation');
legend('Location', 'north');