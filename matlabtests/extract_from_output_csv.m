%% extract_from_output_csv.m — robust IMU extraction for DataFlash CSV
% - Works even if headers are weird (e.g., 'FMT.1' becomes 'FMT_1')
% - Finds IMU rows without assuming the column is named 'FMT'
% - Auto-locates gyro & accel columns by name patterns (with numeric fallback)
% - Plots and builds iddata (placeholder inputs until you log rate setpoints)

clear; clc; close all;

csvFile = 'output.csv';
lpCutHz = [];   % e.g., 35 to low-pass; [] = no filter

%% --- Read CSV (try preserving headers)
T = [];
read_ok = false;
try
    opts = detectImportOptions(csvFile);
    opts.VariableNamingRule = 'preserve';        % keep original headers if supported
    opts = setvaropts(opts, opts.VariableNames, 'Type','char');  % read as text
    T = readtable(csvFile, opts);
    read_ok = true;
catch
end
if ~read_ok
    % Fallback: common delimiters
    try
        T = readtable(csvFile, 'Delimiter',',', 'TextType','string');
    catch
        T = readtable(csvFile, 'Delimiter',';', 'TextType','string');
    end
end

names = string(T.Properties.VariableNames);

% Helper to fetch column by exact or fuzzy name
getcol = @(TT, key) TT.(key); %#ok<NASGU> % (unused, but kept for clarity)

%% --- Find the column that contains the message/type (with value 'IMU')
fmtCol = "";
for nm = names
    try
        vals = string(T.(nm));
        if any(strcmpi(vals, "IMU"))
            fmtCol = nm; break;
        end
    catch
        % non-stringable column, skip
    end
end
if fmtCol == ""
    error('Could not find a column containing the string "IMU".');
end

% Filter to IMU rows
imu = T(strcmpi(string(T.(fmtCol)), 'IMU'), :);
assert(~isempty(imu), 'No IMU rows found.');

imuNames = string(imu.Properties.VariableNames);

%% --- Build time vector (robust)
t = [];
% 1) common numeric microsecond col (often called '128')
if any(imuNames == "128")
    v = str2double(imu.("128"));
    if all(isfinite(v))
        t = (v - v(1))*1e-6;
    end
end
% 2) try any datetime-looking column
if isempty(t)
    for nm = imuNames
        vals = string(imu.(nm));
        if all(strlength(vals) >= 12)
            try
                dt = datetime(vals, 'InputFormat','yyyy-MM-dd HH:mm:ss.SSS');
                if any(~isnat(dt))
                    t = seconds(dt - dt(1));
                    break;
                end
            catch
            end
        end
    end
end
% 3) numeric monotonic fallback
if isempty(t)
    for nm = imuNames
        v = str2double(imu.(nm));
        if any(isfinite(v))
            v = v(:);
            dv = diff(v);
            if nnz(isfinite(dv))>0 && nnz(dv>0) > 0.6*numel(dv)
                % Assume microseconds
                t = (v - v(1))*1e-6;
                break;
            end
        end
    end
end
assert(~isempty(t), 'Could not derive a valid time vector (t).');

%% --- Find gyro & accel columns (pattern-based, with fallback)
canon = lower(imuNames);

% Candidate name patterns (case-insensitive, allow punctuation changes)
gyroCandidatesX = ["gyrx","gyro_x","gyro.x","xgyro","p","fmt.1","fmt_1"];
gyroCandidatesY = ["gyry","gyro_y","gyro.y","ygyro","q","bbnnz"];   % 'BBnNZ' often mangled
gyroCandidatesZ = ["gyrz","gyro_z","gyro.z","zgyro","r","type"];

accCandidatesX  = ["accx","acc_x","accelx","ax","length","accx_mss","acc_x_mss"];
accCandidatesY  = ["accy","acc_y","accely","ay","name","accy_mss","acc_y_mss"];
accCandidatesZ  = ["accz","acc_z","accelz","az","format","accz_mss","acc_z_mss"];

findByPattern = @(cands) localFindCol(canon, imuNames, cands);

gxName = findByPattern(gyroCandidatesX);
gyName = findByPattern(gyroCandidatesY);
gzName = findByPattern(gyroCandidatesZ);

axName = findByPattern(accCandidatesX);
ayName = findByPattern(accCandidatesY);
azName = findByPattern(accCandidatesZ);

% If any missing, try numeric fallback: choose 3 centered, small-mean columns as gyro
if any([gxName=="" , gyName=="" , gzName==""])
    [gxName, gyName, gzName] = localNumericGyroFallback(imu, imuNames);
end
% If any accel missing, try numeric fallback different from chosen gyro
if any([axName=="" , ayName=="" , azName==""])
    [axName, ayName, azName] = localNumericAccelFallback(imu, imuNames, [gxName gyName gzName]);
end

assert(all(~strcmp([gxName gyName gzName], "")), 'Failed to locate 3 gyro columns.');
assert(all(~strcmp([axName ayName azName], "")), 'Failed to locate 3 accel columns.');

% Convert to numeric
gyrX = str2double(imu.(gxName));
gyrY = str2double(imu.(gyName));
gyrZ = str2double(imu.(gzName));
accX = str2double(imu.(axName));
accY = str2double(imu.(ayName));
accZ = str2double(imu.(azName));

% Show what was selected (helps debugging)
fprintf('Selected gyro columns: X="%s", Y="%s", Z="%s"\n', gxName, gyName, gzName);
fprintf('Selected accel columns: X="%s", Y="%s", Z="%s"\n', axName, ayName, azName);

%% --- Clean and (optionally) filter
gyrX = detrend(gyrX,'constant'); gyrY = detrend(gyrY,'constant'); gyrZ = detrend(gyrZ,'constant');
accX = detrend(accX,'constant'); accY = detrend(accY,'constant'); accZ = detrend(accZ,'constant');

dt = diff(t); Ts = median(dt(dt>0 & isfinite(dt)));
if ~isfinite(Ts) || Ts <= 0, Ts = 0.004; end

if ~isempty(lpCutHz) && isfinite(lpCutHz)
    [b,a] = butter(3, min(0.99, lpCutHz/(0.5/Ts)));
    gyrX = filtfilt(b,a,gyrX); gyrY = filtfilt(b,a,gyrY); gyrZ = filtfilt(b,a,gyrZ);
    accX = filtfilt(b,a,accX); accY = filtfilt(b,a,accY); accZ = filtfilt(b,a,accZ);
end

%% --- Plots
figure('Name','Gyroscope (units TBD)');
plot(t, gyrX); hold on; plot(t, gyrY); plot(t, gyrZ); grid on;
xlabel('Time [s]'); ylabel('rate'); legend('gyrX','gyrY','gyrZ','Location','best');
title('IMU Gyro (cleaned)');

figure('Name','Accelerometer (units TBD)');
plot(t, accX); hold on; plot(t, accY); plot(t, accZ); grid on;
xlabel('Time [s]'); ylabel('accel'); legend('accX','accY','accZ','Location','best');
title('IMU Accel (cleaned)');

%% --- System Identification prep (placeholder inputs)
u_roll  = [0; diff(gyrX)/Ts];
u_pitch = [0; diff(gyrY)/Ts];
u_yaw   = [0; diff(gyrZ)/Ts];

roll_id  = iddata(gyrX,  u_roll,  Ts); roll_id.InputName  = "RollCmd";  roll_id.OutputName  = "RollRate";
pitch_id = iddata(gyrY,  u_pitch, Ts); pitch_id.InputName = "PitchCmd"; pitch_id.OutputName = "PitchRate";
yaw_id   = iddata(gyrZ,  u_yaw,   Ts); yaw_id.InputName   = "YawCmd";   yaw_id.OutputName   = "YawRate";

assignin('base','roll_id', roll_id);
assignin('base','pitch_id',pitch_id);
assignin('base','yaw_id',  yaw_id);
assignin('base','Ts', Ts);
assignin('base','t', t);

disp('✓ Data ready. Run:  systemIdentification');
disp('  Import roll_id / pitch_id / yaw_id. Replace placeholder inputs with logged rate setpoints when available.');

%% ==================== Helpers ====================
function nm = localFindCol(canon, originalNames, candidates)
% find a column whose lowercased name contains any candidate (punctuation-insensitive)
    nm = "";
    for c = candidates
        hits = contains(canon, lower(strrep(c,".","_"))) | contains(canon, lower(strrep(c,"_","."))) | contains(canon, lower(c));
        idx = find(hits, 1, 'first');
        if ~isempty(idx)
            nm = originalNames(idx);
            return;
        end
    end
end

function [gx, gy, gz] = localNumericGyroFallback(imu, names)
% Pick three numeric columns that look like small, zero-mean-ish rates
    gx = ""; gy = ""; gz = "";
    cand = [];
    for i = 1:numel(names)
        v = str2double(imu.(names(i)));
        if any(isfinite(v))
            v = v(:);
            if nnz(isfinite(v)) > 0.95*numel(v)
                m = mean(v,'omitnan'); s = std(v,'omitnan');
                % gyro often ~ centered w/ small magnitude
                score = (abs(m) + 1e-12) + 0.1*s;  % prefer small mean, modest std
                cand(end+1,:) = [i, score]; %#ok<AGROW>
            end
        end
    end
    if isempty(cand), return; end
    cand = sortrows(cand, 2, "ascend");
    pick = cand(1:min(3,size(cand,1)),1);
    if numel(pick) >= 3
        gx = names(pick(1)); gy = names(pick(2)); gz = names(pick(3));
    end
end

function [ax, ay, az] = localNumericAccelFallback(imu, names, usedGyro)
% Pick three numeric columns different from gyro, with higher variance (acc often larger swings)
    ax = ""; ay = ""; az = "";
    used = ismember(names, usedGyro);
    cand = [];
    for i = 1:numel(names)
        if used(i), continue; end
        v = str2double(imu.(names(i)));
        if any(isfinite(v))
            v = v(:);
            if nnz(isfinite(v)) > 0.95*numel(v)
                m = mean(v,'omitnan'); s = std(v,'omitnan');
                score = -s + 0.01*abs(m);  % prefer higher std for accel
                cand(end+1,:) = [i, score]; %#ok<AGROW>
            end
        end
    end
    if isempty(cand), return; end
    cand = sortrows(cand, 2, "ascend"); % most negative (largest std) first
    pick = cand(1:min(3,size(cand,1)),1);
    if numel(pick) >= 3
        ax = names(pick(1)); ay = names(pick(2)); az = names(pick(3));
    end
end
