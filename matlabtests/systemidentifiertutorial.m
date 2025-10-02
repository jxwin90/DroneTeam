%% System Identification + Simulink PID Tuning Demo
% STEP 1. Define the "true system" (unknown to identification)
true_sys = tf(5, [0.5 1]);   % G(s) = 5 / (0.5s + 1)

% STEP 2. Generate input signal (step input)
t = 0:0.1:20;           % time vector (0.1s sample, 20s total)
u = ones(size(t));      % step input (1 unit)

% STEP 3. Simulate output (add measurement noise)
y = lsim(true_sys, u, t);          % true system response
y_noisy = y + 0.05*randn(size(y)); % noisy measurement

% STEP 4. Create identification dataset
Ts = 0.1;                        % sampling time
z = iddata(y_noisy, u', Ts);     % iddata object for toolbox

% STEP 5. Identify models
sys_tf = tfest(z, 1, 0);         % 1-pole transfer function
sys_ss = n4sid(z, 1);            % 1-state state-space model
sys_arx = arx(z, [1 1 1]);       % ARX model

% STEP 6. Validate models against data
figure;
compare(z, sys_tf, sys_ss, sys_arx);

% STEP 7. Inspect transfer function model
disp('Identified Transfer Function Model:')
sys_tf

% STEP 8. Send model to PID Tuner
pidTuner(sys_tf,'PIDF');   % Opens PID Tuner with identified system
