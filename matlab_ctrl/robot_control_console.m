%% =========================================================================
%  robot_control_console.m - Giao dien MATLAB dieu khien, Live Stream (He A)
%                            va Burst du lieu SRAM (He B) cho LineRobot ESP32
%  =========================================================================
%  HUONG DAN SU DUNG:
%   1) Ket noi Wi-Fi laptop vao SoftAP cua robot:
%        SSID: LineRobot-AP   |   Pass: 12345678   |   ESP32 IP: 192.168.4.1
%   2) Chay script nay trong MATLAB (R2020b tro len - dung udpport).
%   3) Chon che do trong menu:
%        1: PING & xem trang thai / tham so hien tai
%        2: Cap nhat tham so SMC / chu ky thich nghi / vi chinh diem dung
%        3: Chay HE A (5 cam bien + Wi-Fi live stream 100 Hz)
%        4: Chay HE B (7 cam bien + log SRAM + cho ket noi lai & Burst Wi-Fi)
%        5: Chi tai Burst SRAM (neu xe da dung va dang cho burst)
%        6: Gui lenh DUNG KHAN CAP (CMD_STOP)
%  =========================================================================

clear; clc;

ESP_IP          = "192.168.4.1";
ESP_CMD_PORT    = 8889;
LOCAL_DATA_PORT = 8888;

% --- Ma lenh & ma goi tin (khop 1:1 voi matlab_link.c) ---
CMD_PING        = uint8(1);
CMD_START_A     = uint8(2);
CMD_START_B     = uint8(3);
CMD_STOP        = uint8(4);
CMD_SET_PARAMS  = uint8(5);
CMD_GET_PARAMS  = uint8(6);
CMD_BURST_REQ   = uint8(7);
CMD_BURST_DONE  = uint8(8);

PKT_ACK         = uint8(128); % 0x80
PKT_STATUS      = uint8(129); % 0x81
PKT_PARAMS      = uint8(130); % 0x82
PKT_LIVE_SAMPLE = uint8(131); % 0x83
PKT_BURST_CHUNK = uint8(132); % 0x84

% Mo cong UDP local (don gian, tu dong giai phong khi dong script)
u = udpport("datagram", "IPV4", "LocalPort", LOCAL_DATA_PORT);
cleanupObj = onCleanup(@() clear('u'));
flush(u);

fprintf('==============================================================\n');
fprintf('  LINEROBOT ESP32 - MATLAB CONTROL & DATA ACQUISITION CONSOLE\n');
fprintf('  Target: %s:%d  <-->  Local Port: %d\n', ESP_IP, ESP_CMD_PORT, LOCAL_DATA_PORT);
fprintf('==============================================================\n');

while true
    fprintf('\n--- MENU ---\n');
    fprintf('  [1] PING & Kiem tra trang thai / tham so SMC\n');
    fprintf('  [2] Gui tham so SMC tuy chinh (SET_PARAMS)\n');
    fprintf('  [3] Chay HE A (5 cam bien + Live Stream Wi-Fi + Dem nguoc 3s)\n');
    fprintf('  [4] Chay HE B (7 cam bien + Log SRAM + Cho Burst Wi-Fi)\n');
    fprintf('  [5] Tai Burst SRAM ngay (khi xe dang o trang thai WAIT_BURST)\n');
    fprintf('  [6] Gui lenh DUNG KHAN CAP (CMD_STOP)\n');
    fprintf('  [0] Thoat\n');
    choice = input('Chon [0-6]: ');
    if isempty(choice), continue; end

    switch choice
        case 0
            fprintf('Thoat chuong trinh.\n');
            break;

        case 1
            [ok, st] = ping_robot(u, ESP_IP, ESP_CMD_PORT, CMD_PING, PKT_STATUS, 2.0);
            if ok
                print_status(st);
                send_cmd(u, ESP_IP, ESP_CMD_PORT, CMD_GET_PARAMS, uint8([]));
                [pok, p] = wait_for_packet(u, PKT_PARAMS, 1.0);
                if pok
                    print_params(decode_params(p));
                end
            else
                fprintf('[!] Khong nhan duoc phan hoi tu ESP32. Kiem tra ket noi Wi-Fi LineRobot-AP!\n');
            end

        case 2
            % Doc tham so hien tai truoc de tien sua
            send_cmd(u, ESP_IP, ESP_CMD_PORT, CMD_GET_PARAMS, uint8([]));
            [pok, pbytes] = wait_for_packet(u, PKT_PARAMS, 1.5);
            if pok
                p = decode_params(pbytes);
            else
                p = default_params();
            end
            print_params(p);
            fprintf('Nhan Enter de giu gia tri hien tai, hoac nhap gia tri moi:\n');
            p.v_req           = prompt_num('  v_req [m/s]          ', p.v_req);
            p.beta_smc        = prompt_num('  beta_smc [m/rad]     ', p.beta_smc);
            p.k_smc           = prompt_num('  k_smc [m/s]          ', p.k_smc);
            p.phi_bl          = prompt_num('  phi_bl [m]           ', p.phi_bl);
            p.ki_smc          = prompt_num('  ki_smc [1/s]         ', p.ki_smc);
            p.z_max           = prompt_num('  z_max [m]            ', p.z_max);
            p.w_max           = prompt_num('  w_max [rad/s]        ', p.w_max);
            p.ts_min_s        = prompt_num('  ts_min_s [s] (>=0.05)', p.ts_min_s);
            p.ts_max_s        = prompt_num('  ts_max_s [s] (<0.20) ', p.ts_max_s);
            p.settle_margin_s = prompt_num('  settle_margin_s [s]  ', p.settle_margin_s);
            p.settle_band_mps = prompt_num('  settle_band_mps [m/s]', p.settle_band_mps);
            p.station_trim_m  = prompt_num('  station_trim_m [m]   ', p.station_trim_m);
            p.goal_trim_m     = prompt_num('  goal_trim_m [m]      ', p.goal_trim_m);

            send_cmd(u, ESP_IP, ESP_CMD_PORT, CMD_SET_PARAMS, encode_params(p));
            [aok, pl] = wait_for_packet(u, PKT_PARAMS, 1.5);
            if aok
                fprintf('[OK] ESP32 da ap dung tham so moi:\n');
                print_params(decode_params(pl));
            else
                fprintf('[!] Khong nhan duoc xac nhan SET_PARAMS!\n');
            end

        case 3
            % ===================== HE A: 5 CAM BIEN + LIVE STREAM =====================
            [ok, st] = ping_robot(u, ESP_IP, ESP_CMD_PORT, CMD_PING, PKT_STATUS, 2.0);
            if ~ok
                fprintf('[!] Chua ket noi duoc voi ESP32!\n');
                continue;
            end
            if st.app_state ~= 0
                fprintf('[!] Robot khong o trang thai IDLE (state=%d).\n', st.app_state);
                continue;
            end

            flush(u);
            send_cmd(u, ESP_IP, ESP_CMD_PORT, CMD_START_A, uint8([]));
            [aok, ack_pl] = wait_for_packet(u, PKT_ACK, 1.5);
            if ~aok || ack_pl(2) ~= 0
                fprintf('[!] ESP32 tu choi lenh START_A!\n');
                continue;
            end

            fprintf('[OK] He A da bat dau! Xe dem nguoc 3s roi chay (dong cua so Live de dung khan cap)...\n');
            [samples, final_st] = run_live_capture_A(u, ESP_IP, ESP_CMD_PORT, CMD_STOP, ...
                                                     PKT_LIVE_SAMPLE, PKT_STATUS);
            if ~isempty(samples)
                save_and_plot_run("SystemA_5S", samples, final_st);
            else
                fprintf('[!] Khong thu duoc mau live nao.\n');
            end

        case 4
            % ===================== HE B: 7 CAM BIEN + SRAM BURST =====================
            [ok, st] = ping_robot(u, ESP_IP, ESP_CMD_PORT, CMD_PING, PKT_STATUS, 2.0);
            if ~ok
                fprintf('[!] Chua ket noi duoc voi ESP32!\n');
                continue;
            end
            if st.app_state ~= 0
                fprintf('[!] Robot khong o trang thai IDLE (state=%d).\n', st.app_state);
                continue;
            end

            flush(u);
            send_cmd(u, ESP_IP, ESP_CMD_PORT, CMD_START_B, uint8([]));
            [aok, ack_pl] = wait_for_packet(u, PKT_ACK, 1.5);
            if ~aok || ack_pl(2) ~= 0
                fprintf('[!] ESP32 tu choi lenh START_B!\n');
                continue;
            end

            fprintf('[OK] He B da nhan lenh!\n');
            fprintf('     ESP32 dang tat Wi-Fi -> chuyen sang 7 cam bien -> doi max(3s, t_switch) -> chay & log SRAM.\n');
            fprintf('     Khi xe dung (du 2s dich / 2s mat line / 2s >16mm), Wi-Fi LineRobot-AP se tu bat lai.\n');
            fprintf('     Hay dam bao Windows tu dong ket noi lai vao Wi-Fi "LineRobot-AP"...\n');
            fprintf('     Dang cho ket noi lai tu ESP32 (nhan Ctrl+C de huy)...\n');

            pause(3.0); % Cho ESP32 tat han Wi-Fi truoc khi bat dau ping cho ket noi lai
            st_burst = wait_for_burst_ready(u, ESP_IP, ESP_CMD_PORT, CMD_PING, PKT_STATUS);
            print_status(st_burst);

            samples = download_sram_burst(u, ESP_IP, ESP_CMD_PORT, CMD_BURST_REQ, CMD_BURST_DONE, PKT_BURST_CHUNK);
            if ~isempty(samples)
                save_and_plot_run("SystemB_7S", samples, st_burst);
            else
                fprintf('[!] Khong nhan duoc du lieu Burst.\n');
            end

        case 5
            st_burst = wait_for_burst_ready(u, ESP_IP, ESP_CMD_PORT, CMD_PING, PKT_STATUS);
            print_status(st_burst);
            samples = download_sram_burst(u, ESP_IP, ESP_CMD_PORT, CMD_BURST_REQ, CMD_BURST_DONE, PKT_BURST_CHUNK);
            if ~isempty(samples)
                save_and_plot_run("SystemB_7S", samples, st_burst);
            end

        case 6
            send_cmd(u, ESP_IP, ESP_CMD_PORT, CMD_STOP, uint8([]));
            fprintf('[!] Da gui lenh DUNG KHAN CAP (CMD_STOP).\n');
    end
end

%% ======================== HAM PHU TRO GIAO THUC ========================

function send_cmd(u, ip, port, cmd_id, payload)
    payload = uint8(payload(:)');
    pl_len  = uint16(numel(payload));
    body    = [uint8(cmd_id), typecast(pl_len, 'uint8'), payload];
    cs      = uint8(0);
    for k = 1:numel(body)
        cs = bitxor(cs, body(k));
    end
    pkt = [uint8([170, 85]), body, cs]; % 0xAA, 0x55
    write(u, pkt, "uint8", ip, port);
end

function [valid, pkt_type, payload] = parse_packet(raw)
    valid = false; pkt_type = uint8(0); payload = uint8([]);
    if numel(raw) < 6, return; end
    if raw(1) ~= 170 || raw(2) ~= 85, return; end
    pkt_type = raw(3);
    pl_len   = double(typecast(uint8(raw(4:5)), 'uint16'));
    if numel(raw) ~= 6 + pl_len, return; end
    cs = uint8(0);
    for k = 3:(5 + pl_len)
        cs = bitxor(cs, raw(k));
    end
    if cs ~= raw(6 + pl_len), return; end
    payload = uint8(raw(6:(5 + pl_len)));
    valid   = true;
end

function [ok, payload] = wait_for_packet(u, expected_type, timeout_s)
    ok = false; payload = uint8([]);
    t0 = tic;
    while toc(t0) < timeout_s
        if u.NumDatagramsAvailable > 0
            dg = read(u, 1, "uint8");
            [v, ptype, pl] = parse_packet(uint8(dg.Data));
            if v && ptype == expected_type
                ok = true;
                payload = pl;
                return;
            end
        else
            pause(0.01);
        end
    end
end

function [ok, st] = ping_robot(u, ip, port, CMD_PING, PKT_STATUS, timeout_s)
    flush(u);
    send_cmd(u, ip, port, CMD_PING, uint8([]));
    [ok, pl] = wait_for_packet(u, PKT_STATUS, timeout_s);
    if ok
        st = decode_status(pl);
    else
        st = struct();
    end
end

function st = wait_for_burst_ready(u, ip, port, CMD_PING, PKT_STATUS)
    while true
        try
            send_cmd(u, ip, port, CMD_PING, uint8([]));
            [ok, pl] = wait_for_packet(u, PKT_STATUS, 0.8);
            if ok
                st = decode_status(pl);
                if st.app_state == 3 || st.summary.sample_count > 0
                    fprintf('\n[OK] Da xac nhan ket noi lai voi ESP32! San sang Burst %u mau SRAM.\n', ...
                            st.summary.sample_count);
                    return;
                end
            end
        catch
            % Bo qua loi mang tam thoi khi Windows dang ket noi lai Wi-Fi
        end
        fprintf('.');
        pause(0.5);
    end
end

function [samples, final_st] = run_live_capture_A(u, ip, port, CMD_STOP, PKT_LIVE_SAMPLE, PKT_STATUS)
    max_prealloc = 5000;
    raw_buf      = zeros(52, max_prealloc, 'uint8');
    n_rx         = 0;
    last_t_ms    = -1;
    final_st     = struct();

    fig = figure('Name', 'He A - Live Telemetry (Dong cua so nay de DUNG KHAN CAP)', ...
                 'Color', 'w', 'Position', [120, 140, 860, 480]);
    subplot(2,1,1);
    h_e2 = animatedline('Color', [0 0.45 0.74], 'LineWidth', 1.6);
    hold on; grid on; yline(3, 'r--'); yline(-3, 'r--'); yline(16, 'k:'); yline(-16, 'k:');
    ylim([-20 20]); ylabel('e_{2,filt} (mm)'); title('Sai so bam line thoi gian thuc (He A - 5 cam bien)');

    subplot(2,1,2);
    h_va = animatedline('Color', [0 0.5 0], 'LineWidth', 1.4, 'DisplayName', 'v_A (m/s)');
    h_vb = animatedline('Color', [0.85 0.33 0.1], 'LineWidth', 1.4, 'DisplayName', 'v_B (m/s)');
    hold on; grid on; ylim([-0.1 0.85]); xlabel('Thoi gian (s)'); ylabel('Van toc (m/s)'); legend('Location','northwest');

    t_last = tic;
    while isvalid(fig)
        if u.NumDatagramsAvailable > 0
            dgs = read(u, u.NumDatagramsAvailable, "uint8");
            for i = 1:numel(dgs)
                [v, ptype, pl] = parse_packet(uint8(dgs(i).Data));
                if ~v, continue; end
                if ptype == PKT_LIVE_SAMPLE && numel(pl) == 52
                    t_last = tic;
                    t_ms_cur = double(typecast(pl(1:4), 'uint32'));
                    if t_ms_cur <= last_t_ms
                        continue; % Bo qua goi bi Wi-Fi MAC truyen lap
                    end
                    last_t_ms = t_ms_cur;
                    if n_rx < max_prealloc
                        n_rx = n_rx + 1;
                        raw_buf(:, n_rx) = pl(:);
                        t_s   = t_ms_cur * 1e-3;
                        e2_mm = double(typecast(pl(7:8), 'int16')) * 0.01;
                        va    = double(typecast(pl(27:28), 'int16')) * 1e-3;
                        vb    = double(typecast(pl(31:32), 'int16')) * 1e-3;
                        addpoints(h_e2, t_s, e2_mm);
                        addpoints(h_va, t_s, va);
                        addpoints(h_vb, t_s, vb);
                    end
                elseif ptype == PKT_STATUS && numel(pl) == 48
                    final_st = decode_status(pl);
                    if final_st.app_state == 0
                        fprintf('[OK] He A da ket thuc (stop_reason=%d).\n', final_st.summary.stop_reason);
                        close(fig);
                        break;
                    end
                end
            end
            drawnow limitrate;
        else
            pause(0.005);
            if n_rx > 0 && toc(t_last) > 4.0
                fprintf('[!] Het goi live > 4s, ket thuc thu.\n');
                break;
            end
        end
    end

    if ~isvalid(fig) && isempty(fieldnames(final_st))
        % Nguoi dung dong cua so -> gui lenh STOP khan cap
        send_cmd(u, ip, port, CMD_STOP, uint8([]));
        pause(0.2);
    end

    if n_rx > 0
        samples = decode_samples_matrix(raw_buf(:, 1:n_rx));
    else
        samples = struct([]);
    end
end

function samples = download_sram_burst(u, ip, port, CMD_BURST_REQ, CMD_BURST_DONE, PKT_BURST_CHUNK)
    flush(u);
    % Yeu cau gui toan bo cac chunk (start=0, count=0)
    send_cmd(u, ip, port, CMD_BURST_REQ, typecast(uint16([0, 0]), 'uint8'));

    chunks_map   = containers.Map('KeyType', 'double', 'ValueType', 'any');
    total_chunks = -1;
    t_idle       = tic;

    fprintf('Dang nhan Burst SRAM...');
    while true
        if u.NumDatagramsAvailable > 0
            dgs = read(u, u.NumDatagramsAvailable, "uint8");
            for i = 1:numel(dgs)
                [v, ptype, pl] = parse_packet(uint8(dgs(i).Data));
                if v && ptype == PKT_BURST_CHUNK && numel(pl) >= 6
                    c_idx = double(typecast(pl(1:2), 'uint16'));
                    c_tot = double(typecast(pl(3:4), 'uint16'));
                    c_cnt = double(typecast(pl(5:6), 'uint16'));
                    total_chunks = c_tot;
                    if c_tot == 0
                        fprintf(' (Bo dem rong)\n');
                        send_cmd(u, ip, port, CMD_BURST_DONE, uint8([]));
                        samples = struct([]);
                        return;
                    end
                    if numel(pl) == 6 + 52 * c_cnt
                        chunks_map(c_idx) = reshape(pl(7:end), 52, c_cnt);
                        t_idle = tic;
                    end
                end
            end
        else
            pause(0.01);
        end

        if total_chunks > 0 && chunks_map.Count >= total_chunks
            break;
        end
        if toc(t_idle) > 1.2
            if total_chunks <= 0
                fprintf('\n[!] Timeout nhan Burst!\n');
                samples = struct([]);
                return;
            end
            % Yeu cau gui lai cac chunk bi thieu (zero-loss retransmission)
            missing = setdiff(0:(total_chunks-1), cell2mat(keys(chunks_map)));
            if isempty(missing), break; end
            fprintf('\n  [Retry] Thieu %d chunk -> yeu cau gui lai...', numel(missing));
            for m = missing
                send_cmd(u, ip, port, CMD_BURST_REQ, typecast(uint16([m, 1]), 'uint8'));
                pause(0.02);
            end
            t_idle = tic;
        end
    end

    % Xac nhan hoan tat de ESP32 giai phong SRAM ve IDLE
    send_cmd(u, ip, port, CMD_BURST_DONE, uint8([]));

    % Ghep cac chunk theo dung thu tu
    all_raw = [];
    for c = 0:(total_chunks - 1)
        all_raw = [all_raw, chunks_map(c)]; %#ok<AGROW>
    end
    fprintf(' HOAN TAT (%d chunks, %d mau)!\n', total_chunks, size(all_raw, 2));
    samples = decode_samples_matrix(all_raw);
end

function S = decode_samples_matrix(raw)
    % raw: 52 x N uint8 -> struct mang cac tin hieu don vi vat ly
    N = size(raw, 2);
    S.t_s             = double(typecast(reshape(raw(1:4,:),   [], 1), 'uint32')) * 1e-3;
    S.e2_raw_mm       = double(typecast(reshape(raw(5:6,:),   [], 1), 'int16'))  * 0.01;
    S.e2_filt_mm      = double(typecast(reshape(raw(7:8,:),   [], 1), 'int16'))  * 0.01;
    S.e3_hat_deg      = double(typecast(reshape(raw(9:10,:),  [], 1), 'int16'))  * 0.01;
    S.c_hat_radm      = double(typecast(reshape(raw(11:12,:), [], 1), 'int16'))  * 1e-3;
    S.sigma_mm        = double(typecast(reshape(raw(13:14,:), [], 1), 'int16'))  * 0.01;
    S.z_int_mm        = double(typecast(reshape(raw(15:16,:), [], 1), 'int16'))  * 0.01;
    S.v_cmd_mps       = double(typecast(reshape(raw(17:18,:), [], 1), 'int16'))  * 1e-3;
    S.w_cmd_rads      = double(typecast(reshape(raw(19:20,:), [], 1), 'int16'))  * 1e-3;
    S.v_act_mps       = double(typecast(reshape(raw(21:22,:), [], 1), 'int16'))  * 1e-3;
    S.w_act_rads      = double(typecast(reshape(raw(23:24,:), [], 1), 'int16'))  * 1e-3;
    S.va_ref_mps      = double(typecast(reshape(raw(25:26,:), [], 1), 'int16'))  * 1e-3;
    S.va_act_mps      = double(typecast(reshape(raw(27:28,:), [], 1), 'int16'))  * 1e-3;
    S.vb_ref_mps      = double(typecast(reshape(raw(29:30,:), [], 1), 'int16'))  * 1e-3;
    S.vb_act_mps      = double(typecast(reshape(raw(31:32,:), [], 1), 'int16'))  * 1e-3;
    S.pwm_a_pct       = double(typecast(reshape(raw(33:34,:), [], 1), 'int16'))  * 0.01;
    S.pwm_b_pct       = double(typecast(reshape(raw(35:36,:), [], 1), 'int16'))  * 0.01;
    S.s_enc_m         = double(typecast(reshape(raw(37:38,:), [], 1), 'uint16')) * 1e-3;
    S.total_i         = double(typecast(reshape(raw(39:40,:), [], 1), 'uint16')) * 1e-3;
    S.inner_jitter_us = double(typecast(reshape(raw(41:42,:), [], 1), 'uint16'));
    S.norm_s          = double(raw(43:49, :)'); % N x 7
    flags             = raw(50, :)';
    S.mission_state   = double(bitand(flags, uint8(15)));
    S.line_ok         = logical(bitand(flags, uint8(16)));
    S.y_gn_active     = logical(bitand(flags, uint8(32)));
    S.smc_updated     = logical(bitand(flags, uint8(64)));
    S.marker_seen     = logical(bitand(flags, uint8(128)));
    S.dt_smc_ms       = double(raw(51, :)');
    S.load_class      = double(raw(52, :)');
    S.N               = N;
end

function st = decode_status(pl)
    st.app_state      = pl(1);
    st.sensor_profile = pl(2);
    st.calib_valid    = pl(3);
    st.sta_connected  = pl(4);
    s = pl(5:48);
    st.summary.run_mode        = s(1);
    st.summary.sensor_profile  = s(2);
    st.summary.stop_reason     = s(3);
    st.summary.load_class_kg   = s(4);
    st.summary.sample_count    = typecast(s(5:8),   'uint32');
    st.summary.run_duration_ms = typecast(s(9:12),  'uint32');
    st.summary.load_measured_g = typecast(s(13:16), 'single');
    st.summary.max_abs_e2_mm   = typecast(s(17:20), 'single');
    st.summary.rms_e2_mm       = typecast(s(21:24), 'single');
    st.summary.station_stop_s_m= typecast(s(25:28), 'single');
    st.summary.goal_stop_s_m   = typecast(s(29:32), 'single');
    st.summary.goal_creep_mm   = typecast(s(33:36), 'single');
    st.summary.avg_dt_smc_ms   = typecast(s(37:40), 'single');
    st.summary.y_gn_calls      = typecast(s(41:44), 'uint32');
end

function p = decode_params(pl)
    f = double(typecast(uint8(pl(1:52)), 'single'));
    p.v_req           = f(1);  p.beta_smc        = f(2);  p.k_smc           = f(3);
    p.phi_bl          = f(4);  p.ki_smc          = f(5);  p.z_max           = f(6);
    p.w_max           = f(7);  p.ts_min_s        = f(8);  p.ts_max_s        = f(9);
    p.settle_margin_s = f(10); p.settle_band_mps = f(11);
    p.station_trim_m  = f(12); p.goal_trim_m     = f(13);
end

function pl = encode_params(p)
    arr = single([p.v_req, p.beta_smc, p.k_smc, p.phi_bl, p.ki_smc, p.z_max, ...
                  p.w_max, p.ts_min_s, p.ts_max_s, p.settle_margin_s, ...
                  p.settle_band_mps, p.station_trim_m, p.goal_trim_m]);
    pl  = typecast(arr, 'uint8');
end

function p = default_params()
    p.v_req = 0.50; p.beta_smc = 0.025; p.k_smc = 0.19; p.phi_bl = 0.018;
    p.ki_smc = 0.95; p.z_max = 0.008; p.w_max = 2.60;
    p.ts_min_s = 0.055; p.ts_max_s = 0.090; p.settle_margin_s = 0.010; p.settle_band_mps = 0.035;
    p.station_trim_m = 0.0; p.goal_trim_m = 0.0;
end

function val = prompt_num(label, cur)
    str = input(sprintf('%s [%.4f]: ', label, cur), 's');
    if isempty(str), val = cur; else, val = str2double(str); if isnan(val), val = cur; end, end
end

function print_status(st)
    states  = {'IDLE', 'RUNNING_A', 'RUNNING_B', 'WAIT_BURST', 'CALIB'};
    reasons = {'NONE', 'GOAL_DONE_2S', 'LINE_LOST_2S', 'ERR_OVER_16MM_2S', 'MANUAL', 'FAULT'};
    sname = states{min(double(st.app_state)+1, numel(states))};
    rname = reasons{min(double(st.summary.stop_reason)+1, numel(reasons))};
    fprintf('  State=%s | Profile=%dS | NVS Calib=%d | Samples=%u | Stop=%s\n', ...
            sname, st.sensor_profile, st.calib_valid, st.summary.sample_count, rname);
    if st.summary.run_duration_ms > 0
        fprintf('  Last Run: dur=%.2fs | load=%.0fg (%dkg) | max|e2|=%.2fmm | RMS=%.2fmm | avg_dt_smc=%.1fms\n', ...
                double(st.summary.run_duration_ms)/1000, st.summary.load_measured_g, ...
                st.summary.load_class_kg, st.summary.max_abs_e2_mm, st.summary.rms_e2_mm, ...
                st.summary.avg_dt_smc_ms);
        fprintf('  Stopping: station_s=%.3fm (ref 2.000m) | goal_s=%.3fm (ref 4.709m, creep=%.1fmm)\n', ...
                st.summary.station_stop_s_m, st.summary.goal_stop_s_m, st.summary.goal_creep_mm);
    end
end

function print_params(p)
    fprintf('  SMC Params: v_req=%.2f | beta=%.4f k=%.3f phi=%.4f ki=%.2f z_max=%.4f w_max=%.2f\n', ...
            p.v_req, p.beta_smc, p.k_smc, p.phi_bl, p.ki_smc, p.z_max, p.w_max);
    fprintf('  Scheduler : Ts=[%.0f..%.0f]ms margin=%.0fms band=%.3fm/s | Trim: station=%+.1fmm goal=%+.1fmm\n', ...
            p.ts_min_s*1e3, p.ts_max_s*1e3, p.settle_margin_s*1e3, p.settle_band_mps, ...
            p.station_trim_m*1e3, p.goal_trim_m*1e3);
end

function save_and_plot_run(prefix, S, st)
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    prefix_chr = char(prefix);

    % Tach prefix ("SystemA_5S" -> "SystemA" va "5S") de tao thu muc co so thu tu tang dan:
    % vi du: SystemA_1_5S_20261127_182807, SystemA_2_5S_..., SystemB_1_7S_...
    tok = regexp(prefix_chr, '^([^_]+)_(.+)$', 'tokens', 'once');
    if ~isempty(tok)
        sys_part  = tok{1};
        sens_part = tok{2};
    else
        sys_part  = prefix_chr;
        sens_part = 'RUN';
    end

    run_idx = next_run_index(pwd, sys_part);
    out_dir = fullfile(pwd, sprintf('%s_%d_%s_%s', sys_part, run_idx, sens_part, stamp));
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    % Ten file du lieu ben trong thu muc giu nguyen nhu hien gio
    mat_file = fullfile(out_dir, sprintf('%s_%s.mat', prefix_chr, stamp));
    csv_file = fullfile(out_dir, sprintf('%s_%s.csv', prefix_chr, stamp));
    save(mat_file, 'S', 'st');

    T = table(S.t_s, S.e2_raw_mm, S.e2_filt_mm, S.e3_hat_deg, S.c_hat_radm, ...
              S.sigma_mm, S.z_int_mm, S.v_cmd_mps, S.w_cmd_rads, S.v_act_mps, S.w_act_rads, ...
              S.va_ref_mps, S.va_act_mps, S.vb_ref_mps, S.vb_act_mps, S.pwm_a_pct, S.pwm_b_pct, ...
              S.s_enc_m, S.total_i, S.inner_jitter_us, S.mission_state, S.dt_smc_ms, S.load_class, ...
              S.norm_s(:,1), S.norm_s(:,2), S.norm_s(:,3), S.norm_s(:,4), ...
              S.norm_s(:,5), S.norm_s(:,6), S.norm_s(:,7), ...
              'VariableNames', {'t_s','e2_raw_mm','e2_filt_mm','e3_hat_deg','c_hat_radm', ...
              'sigma_mm','z_int_mm','v_cmd_mps','w_cmd_rads','v_act_mps','w_act_rads', ...
              'va_ref_mps','va_act_mps','vb_ref_mps','vb_act_mps','pwm_a_pct','pwm_b_pct', ...
              's_enc_m','total_i','jitter_us','state','dt_smc_ms','load_kg', ...
              'S1','S2','S3','S4','S5','S6','S7'});
    writetable(T, csv_file);

    % --- Figure 1: Sai so bam line e2 ---
    f1 = figure('Name', [prefix_chr ' - Sai so bam line e2'], 'Color', 'w', 'Position', [60, 480, 740, 380]);
    hold on; grid on;
    plot(S.t_s, S.e2_raw_mm, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.9);
    plot(S.t_s, S.e2_filt_mm, 'b-', 'LineWidth', 1.6);
    plot(S.t_s, double(S.y_gn_active) * 2.5, 'g-', 'LineWidth', 1.2);
    yline(3, 'r--', 'LineWidth', 1.2); yline(-3, 'r--', 'LineWidth', 1.2);
    yline(16, 'k:', 'LineWidth', 1.0); yline(-16, 'k:', 'LineWidth', 1.0);
    xlabel('Thoi gian (s)'); ylabel('e_2 (mm)'); ylim([-20 20]);
    title('Sai so lech ngang e_2 (SMC frame: + = Trai)');
    legend('e_{2,raw}', 'e_{2,filt} (Alpha-Beta)', 'Co tach nhanh Y', '\pm3 mm', '\pm16 mm', 'Location', 'best');
    export_fig_png_svg(f1, fullfile(out_dir, [prefix_chr ' - Sai so bam line e2']));

    % --- Figure 2: Luenberger Observer & Mat truot SMC ---
    f2 = figure('Name', [prefix_chr ' - Observer & Sliding Surface'], 'Color', 'w', 'Position', [820, 480, 720, 420]);
    subplot(2,1,1);
    yyaxis left; plot(S.t_s, S.e3_hat_deg, 'b-', 'LineWidth', 1.4); ylabel('e_{3,hat} (deg)'); grid on;
    yyaxis right; plot(S.t_s, S.c_hat_radm, 'r--', 'LineWidth', 1.2); ylabel('c_{hat} (rad/m)');
    title('Bo quan sat Luenberger: e_{3,hat} va do cong c_{hat}');
    subplot(2,1,2);
    plot(S.t_s, S.sigma_mm, 'b-', 'LineWidth', 1.5); hold on; grid on;
    plot(S.t_s, S.z_int_mm, 'm--', 'LineWidth', 1.3);
    xlabel('Thoi gian (s)'); ylabel('mm'); legend('\sigma (mm)', 'z_{int} (mm)', 'Location', 'best');
    title('Mat truot \sigma va thanh phan tich phan z_{int}');
    export_fig_png_svg(f2, fullfile(out_dir, [prefix_chr ' - Observer & Sliding Surface']));

    % --- Figure 3: Vong trong (2 banh & PWM) ---
    f3 = figure('Name', [prefix_chr ' - Inner Loop Tracking'], 'Color', 'w', 'Position', [60, 40, 740, 420]);
    subplot(2,1,1);
    plot(S.t_s, S.va_ref_mps, 'b--', S.t_s, S.va_act_mps, 'b-', ...
         S.t_s, S.vb_ref_mps, 'r--', S.t_s, S.vb_act_mps, 'r-', 'LineWidth', 1.3);
    grid on; ylabel('m/s'); legend('v_{A,ref}','v_{A,act}','v_{B,ref}','v_{B,act}','Location','best');
    title('Dap ung vong trong 2-DOF PI (200 Hz)');
    subplot(2,1,2);
    plot(S.t_s, S.pwm_a_pct, 'b-', S.t_s, S.pwm_b_pct, 'r-', 'LineWidth', 1.3);
    grid on; xlabel('Thoi gian (s)'); ylabel('PWM (%)'); legend('PWM A','PWM B');
    export_fig_png_svg(f3, fullfile(out_dir, [prefix_chr ' - Inner Loop Tracking']));

    % --- Figure 4: Chu ky SMC thich nghi & Van toc than xe ---
    f4 = figure('Name', [prefix_chr ' - Adaptive SMC & Odometry'], 'Color', 'w', 'Position', [820, 40, 720, 420]);
    subplot(2,1,1);
    stairs(S.t_s, S.dt_smc_ms, 'k-', 'LineWidth', 1.4); hold on; grid on;
    yline(50, 'b--'); yline(200, 'r--');
    ylabel('dt_{SMC} (ms)'); ylim([0 220]);
    title('Chu ky cap nhat vong ngoai SMC thich nghi theo xac lap vong trong');
    subplot(2,1,2);
    yyaxis left; plot(S.t_s, S.v_cmd_mps, 'k--', S.t_s, S.v_act_mps, 'b-', 'LineWidth', 1.3); ylabel('v (m/s)'); grid on;
    yyaxis right; plot(S.t_s, S.w_cmd_rads, 'm:', S.t_s, S.w_act_rads, 'r-', 'LineWidth', 1.2); ylabel('\omega (rad/s)');
    xlabel('Thoi gian (s)'); legend('v_{cmd}','v_{act}','w_{cmd}','w_{act}','Location','best');
    export_fig_png_svg(f4, fullfile(out_dir, [prefix_chr ' - Adaptive SMC & Odometry']));

    % --- Figure 5: Ban do 7 mat TCRT5000 & total_I ---
    f5 = figure('Name', [prefix_chr ' - 7-Sensor Array Heatmap'], 'Color', 'w', 'Position', [440, 220, 760, 400]);
    subplot(2,1,1);
    imagesc(S.t_s, 1:7, S.norm_s'); colormap(flipud(gray)); colorbar;
    yticks(1:7); yticklabels({'S1(-27)','S2(-18)','S3(-9)','S4(0)','S5(+9)','S6(+18)','S7(+27)'});
    ylabel('Cam bien'); title('Cuong do phan xa 7 mat TCRT5000 (0% trang -> 100% den)');
    subplot(2,1,2);
    plot(S.t_s, S.total_i, 'b-', 'LineWidth', 1.3); hold on; grid on;
    plot(S.t_s, S.mission_state * 0.5, 'r--', 'LineWidth', 1.1);
    xlabel('Thoi gian (s)'); ylabel('total_I / 0.5*State');
    legend('total\_I', '0.5 \times Mission State', 'Location', 'best');
    export_fig_png_svg(f5, fullfile(out_dir, [prefix_chr ' - 7-Sensor Array Heatmap']));

    fprintf('[SAVED] Thu muc: %s\n', out_dir);
    fprintf('        Du lieu: .mat, .csv (%d samples) + 5 do thi (.png & .svg)\n', S.N);
end

function idx = next_run_index(base_dir, sys_part)
    % Tim so thu tu lon nhat cua cac thu muc dang "<sys_part>_<N>_*" trong base_dir
    d = dir(base_dir);
    max_n = 0;
    pat = ['^' regexptranslate('escape', sys_part) '_(\d+)_'];
    for k = 1:numel(d)
        if d(k).isdir
            m = regexp(d(k).name, pat, 'tokens', 'once');
            if ~isempty(m)
                n = str2double(m{1});
                if ~isnan(n) && n > max_n
                    max_n = n;
                end
            end
        end
    end
    idx = max_n + 1;
end

function export_fig_png_svg(fig_handle, base_path)
    drawnow;
    png_path = [base_path '.png'];
    svg_path = [base_path '.svg'];
    try
        exportgraphics(fig_handle, png_path, 'Resolution', 300);
    catch
        print(fig_handle, png_path, '-dpng', '-r300');
    end
    try
        print(fig_handle, svg_path, '-dsvg', '-vector');
    catch
        try
            print(fig_handle, svg_path, '-dsvg');
        catch
            saveas(fig_handle, svg_path);
        end
    end
end
