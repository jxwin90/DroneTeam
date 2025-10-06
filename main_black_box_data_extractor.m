%% ====== Load ArduPilot Log (CSV exported from Mission Planner) ======
opts = detectImportOptions('final.csv', 'Delimiter', ';');
for k = 1:numel(opts.VariableTypes)
    opts.VariableTypes{k} = 'char';   % read all columns as text
end
data = readtable('final.csv', opts);

%% ====== Identify available message types ======
disp('Available message types:');
disp(unique(data.Column3));

%% ====== Helper function for safe extraction ======
extract_col = @(rows, colnum) str2double(data{rows, min(colnum, width(data))});

%% ====== Extract relevant message rows ======
imu_rows   = strcmp(data.Column3, 'IMU');    % filtered IMU
gyr_rows   = strcmp(data.Column3, 'GYR');    % raw gyro
acc_rows   = strcmp(data.Column3, 'ACC');    % raw accel
att_rows   = strcmp(data.Column3, 'ATT');    % attitude
rcin_rows  = strcmp(data.Column3, 'RCIN');   % pilot input
rcout_rows = strcmp(data.Column3, 'RCOU');   % motor output

%% ====== IMU (Filtered) ======
IMU.GyrX = extract_col(imu_rows,6);
IMU.GyrY = extract_col(imu_rows,7);
IMU.GyrZ = extract_col(imu_rows,8);
IMU.AccX = extract_col(imu_rows,9);
IMU.AccY = extract_col(imu_rows,10);
IMU.AccZ = extract_col(imu_rows,11);

%% ====== RAW IMU Equivalent (Combine GYR + ACC) ======
RAWIMU.GyrX = extract_col(gyr_rows,6);
RAWIMU.GyrY = extract_col(gyr_rows,7);
RAWIMU.GyrZ = extract_col(gyr_rows,8);
RAWIMU.AccX = extract_col(acc_rows,6);
RAWIMU.AccY = extract_col(acc_rows,7);
RAWIMU.AccZ = extract_col(acc_rows,8);

%% ====== Attitude (actual vs desired) ======
ATT.Roll  = extract_col(att_rows,6);
ATT.Pitch = extract_col(att_rows,7);
ATT.Yaw   = extract_col(att_rows,8);
ATT.DesRoll  = extract_col(att_rows,9);
ATT.DesPitch = extract_col(att_rows,10);
ATT.DesYaw   = extract_col(att_rows,11);

%% ====== RC Inputs ======
RCIN.Ch1 = extract_col(rcin_rows,6);  % Roll input
RCIN.Ch2 = extract_col(rcin_rows,7);  % Pitch input
RCIN.Ch3 = extract_col(rcin_rows,8);  % Throttle input
RCIN.Ch4 = extract_col(rcin_rows,9);  % Yaw input

%% ====== RC Outputs ======
RCOUT.M1 = extract_col(rcout_rows,6);
RCOUT.M2 = extract_col(rcout_rows,7);
RCOUT.M3 = extract_col(rcout_rows,8);
RCOUT.M4 = extract_col(rcout_rows,9);

%% ====== Plot IMU ======
figure;
subplot(2,1,1);
plot(IMU.GyrX,'r'); hold on; plot(IMU.GyrY,'g'); plot(IMU.GyrZ,'b');
title('IMU Gyroscope'); legend({'GyrX','GyrY','GyrZ'});
subplot(2,1,2);
plot(IMU.AccX,'r'); hold on; plot(IMU.AccY,'g'); plot(IMU.AccZ,'b');
title('IMU Accelerometer'); legend({'AccX','AccY','AccZ'});

%% ====== Plot RAW IMU ======
figure;
subplot(2,1,1);
plot(RAWIMU.GyrX,'r'); hold on; plot(RAWIMU.GyrY,'g'); plot(RAWIMU.GyrZ,'b');
title('RAW IMU Gyroscope'); legend({'GyrX','GyrY','GyrZ'});
subplot(2,1,2);
plot(RAWIMU.AccX,'r'); hold on; plot(RAWIMU.AccY,'g'); plot(RAWIMU.AccZ,'b');
title('RAW IMU Accelerometer'); legend({'AccX','AccY','AccZ'});

%% ====== Plot Attitude (desired vs actual) ======
figure;
subplot(3,1,1);
plot(ATT.Roll,'b'); hold on; plot(ATT.DesRoll,'r--');
title('Roll Response'); legend({'Actual','Desired'});
subplot(3,1,2);
plot(ATT.Pitch,'b'); hold on; plot(ATT.DesPitch,'r--');
title('Pitch Response'); legend({'Actual','Desired'});
subplot(3,1,3);
plot(ATT.Yaw,'b'); hold on; plot(ATT.DesYaw,'r--');
title('Yaw Response'); legend({'Actual','Desired'});

%% ====== Plot RC Inputs and Outputs ======
figure;
subplot(2,1,1);
plot(RCIN.Ch1,'r'); hold on; plot(RCIN.Ch2,'g'); plot(RCIN.Ch3,'b'); plot(RCIN.Ch4,'k');
title('RC Inputs (Pilot Commands)'); legend({'Roll','Pitch','Throttle','Yaw'});
subplot(2,1,2);
plot(RCOUT.M1,'r'); hold on; plot(RCOUT.M2,'g'); plot(RCOUT.M3,'b'); plot(RCOUT.M4,'k');
title('RC Outputs (Motor PWMs)'); legend({'M1','M2','M3','M4'});

%% ====== System Identification Preparation (Safe & Robust) ======
Ts = 0.02;  % 50 Hz typical SITL rate

% Ensure vectors
ATT.Roll  = ATT.Roll(:);
ATT.Pitch = ATT.Pitch(:);
ATT.Yaw   = ATT.Yaw(:);
RCIN.Ch1  = RCIN.Ch1(:);
RCIN.Ch2  = RCIN.Ch2(:);
RCIN.Ch4  = RCIN.Ch4(:);

% Skip if too few points
if numel(RCIN.Ch1) < 2
    warning('⚠️ RCIN data missing or too short – skipping interpolation.');
    return;
end

% Interpolation with safety
t_rc  = linspace(0, 1, length(RCIN.Ch1));
t_att = linspace(0, 1, length(ATT.Roll));

RCIN_resampled.Ch1 = interp1(t_rc, RCIN.Ch1, t_att, 'linear', 'extrap');
RCIN_resampled.Ch2 = interp1(t_rc, RCIN.Ch2, t_att, 'linear', 'extrap');
RCIN_resampled.Ch4 = interp1(t_rc, RCIN.Ch4, t_att, 'linear', 'extrap');

% Match lengths
minLen = min([length(ATT.Roll), length(RCIN_resampled.Ch1)]);
ATT_trim.Roll  = ATT.Roll(1:minLen);
ATT_trim.Pitch = ATT.Pitch(1:minLen);
ATT_trim.Yaw   = ATT.Yaw(1:minLen);
RCIN_trim.Ch1  = RCIN_resampled.Ch1(1:minLen);
RCIN_trim.Ch2  = RCIN_resampled.Ch2(1:minLen);
RCIN_trim.Ch4  = RCIN_resampled.Ch4(1:minLen);

% Build iddata objects
id_roll  = iddata(ATT_trim.Roll,  RCIN_trim.Ch1, Ts, 'Name', 'Roll Axis');
id_pitch = iddata(ATT_trim.Pitch, RCIN_trim.Ch2, Ts, 'Name', 'Pitch Axis');
id_yaw   = iddata(ATT_trim.Yaw,   RCIN_trim.Ch4, Ts, 'Name', 'Yaw Axis');

disp('✅ iddata objects created: id_roll, id_pitch, id_yaw');
disp('➡️  Try: sys_roll = tfest(id_roll, 2); bode(sys_roll)');