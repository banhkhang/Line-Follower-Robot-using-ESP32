function export_mat_to_csv(mat_filename, csv_filename)
   % export_mat_to_csv('dataTocDo', 'dataTocDo.csv')
   % export_mat_to_csv('dataError', 'dataError.csv')
    S = load(mat_filename);
    fields = fieldnames(S);
    raw = S.(fields{1});

    if isa(raw, 'Simulink.SimulationData.Dataset')
        ts = raw{1}.Values;
        t = ts.Time(:);
        y = squeeze(ts.Data);
    elseif isa(raw, 'timeseries')
        t = raw.Time(:);
        y = squeeze(raw.Data);
    elseif isstruct(raw) && isfield(raw, 'signals')
        t = raw.time(:);
        y = raw.signals(1).values;
    elseif isnumeric(raw)
        % Định dạng mặc định của khối "To File" (hàng 1 là time, các hàng sau là data)
        if size(raw, 1) < size(raw, 2)
            raw = raw.';
        end
        t = raw(:, 1);
        y = raw(:, 2:end);
    else
        error('Không nhận diện được kiểu dữ liệu: %s', class(raw));
    end

    if size(y, 1) ~= numel(t)
        y = y.';
    end

    T = array2table([t, y]);
    T.Properties.VariableNames{1} = 'Time_s';
    writetable(T, csv_filename);
    fprintf('Đã xuất %s (%d dòng x %d cột)\n', csv_filename, height(T), width(T));
end