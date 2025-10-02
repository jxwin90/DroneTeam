%% Auto-build Closed Loop PID + Plant Simulink Model
modelName = 'ClosedLoopPID';
new_system(modelName);      % create new model
open_system(modelName);     % open it

%% Add blocks
add_block('simulink/Sources/Step', [modelName '/Step'], 'Position', [50 100 80 130]);
add_block('simulink/Math Operations/Sum', [modelName '/Sum'], 'Position', [120 95 140 135], 'Inputs', '+-');
add_block('simulink/Continuous/PID Controller', [modelName '/PID'], 'Position', [180 80 240 150]);
add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant'], 'Position', [300 80 380 150]);
add_block('simulink/Sinks/Scope', [modelName '/Scope'], 'Position', [450 95 480 125]);

%% Connect lines
add_line(modelName, 'Step/1', 'Sum/1');
add_line(modelName, 'Sum/1', 'PID/1');
add_line(modelName, 'PID/1', 'Plant/1');
add_line(modelName, 'Plant/1', 'Scope/1');
add_line(modelName, 'Plant/1', 'Sum/2');

%% Configure blocks
% Example plant: 5/(0.5s+1)
set_param([modelName '/Plant'], 'Numerator', '5', 'Denominator', '[0.5 1]');

% Use exported PID gains from workspace (variable C)
set_param([modelName '/PID'], 'P', 'C.Kp', 'I', 'C.Ki', 'D', 'C.Kd', 'N', 'C.N');

% Step input settings
set_param([modelName '/Step'], 'Time', '0', 'Before', '0', 'After', '1');

%% Save and simulate
save_system(modelName);
set_param(modelName, 'StopTime', '10');   % simulate for 10s
sim(modelName);

%asdfaf