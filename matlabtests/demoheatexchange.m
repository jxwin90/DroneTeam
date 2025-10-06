%% Step A: Load the data
load iddemo_heatexchanger_data   % loads variables: pt, ct, Ts

%% Step B: Create iddata
data = iddata(pt, ct, Ts);
data.InputName  = '\Delta CTemp';
data.OutputName = '\Delta PTemp';
data.TimeUnit   = 'minutes';

% Plot data
plot(data)

%% Step C: Estimate a basic transfer function (1 pole, 0 zeros, allow delay)
sysTF = tfest(data, 1, 0, nan);
sysTF  % show the identified model

%% Step D: Assess model fit / residuals
resid(sysTF, data);
figure;
compare(data, sysTF);

%% Step E: Build an initial guess model using prior knowledge
sysInit = idtf(NaN, [1 NaN], 'ioDelay', NaN);
% Constrain the structure (gain positive, denominator >= 0, etc.)
sysInit.Structure.num.Value   = 1;
sysInit.Structure.num.Minimum = 0;
sysInit.Structure.den.Value   = [1 1];
sysInit.Structure.den.Minimum = [0 0];
sysInit.Structure.ioDelay.Value = 0.2;  % for example
sysInit.Structure.ioDelay.Minimum = 0;
sysInit.Structure.ioDelay.Maximum = 1;

sysTF_initialized = tfest(data, sysInit);

%% Step F: Estimate a process model with known form (P1D = first order + delay)
sysP1D = procest(data, 'P1D');

%% Optional Step G: Add disturbance/noise model (e.g. ARMA) to account for residuals
opt = procestOptions('DisturbanceModel','ARMA1');
sysInit2 = idproc('P1D','TimeUnit','minutes');
% set constraints on sysInit2 as needed (gain, delay bounds, etc.)
sysP1D_noise = procest(data, sysInit2, opt);

%% Step H: Compare all models
compare(data, sysTF, sysTF_initialized, sysP1D, sysP1D_noise);
