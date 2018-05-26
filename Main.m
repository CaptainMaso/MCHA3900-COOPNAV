clear
clear variables;

addpath('QUAD');
addpath('WAMV');
addpath('AUV');
addpath('Sensors');
addpath('Common');
addpath('Estimators');

warning('off', 'MATLAB:legend:IgnoringExtraEntries');

global param map data
%% Parameter Initialisation
param.enabled = [1,1,1]; % AUV, WAMV, QUAD

% -- Universal Parameters
% --- World Parameters
param.g = 9.81;

% --- Sensor Parameters
param.sensor_sample_rate = 100;  % Sample rate (Hz)
param.IMU.samplerate = 100;

% ---- LPS parameters
param.LPS.sigma = 0.1;          % Noise on LPS data (m)
param.LPS.datalength = 1;

% ---- Visual Bearing (VB) parameters
param.VB.sigma      = eye(3)*0.01;      % Noise on VB data (rad)
param.VB.datalength = 3;

% ---- IMU parameters
param.IMU.acc_sigma = eye(3)*0.01;      % Noise on accelerometer data (m/s^2)
param.IMU.gyro_sigma = eye(3)*0.01;     % Noise on gyro data (rad/s)
param.IMU.gyro_bias  = [0.4, -0.2, 0.6]';
param.IMU.magn_sigma = eye(3)*0.01;
param.IMU.datalength = 9;

% ---- GPS parameters
param.GPS.sigma      = eye(3)*5;       % Noise on gps data (m)
param.GPS.datalength = 3;

% ---- HAP parameters
param.HAP.sigma      = eye(3)*3;       % Noise on HAP data (m)
param.HAP.datalength = 3;

% --- Simulation Parameters
param.tf = 50;

% --- Vehicle Uncertainties
param.AUV.SQeta = diag([1 1 1, deg2rad([10 10 10])]);
param.AUV.SQnu  = diag([0.5 0.5 0.5, deg2rad([5 5 5])]);

param.WAMV.SQeta = diag([1 1 1, deg2rad([10 10 10])]);
param.WAMV.SQnu  = diag([0.5 0.5 0.5, deg2rad([5 5 5])]);

param.QUAD.SQeta = diag([1 1 1, deg2rad([10 10 10])]);
param.QUAD.SQnu  = diag([0.5 0.5 0.5, deg2rad([5 5 5])]);

param.SQbias     = diag(deg2rad([1 1 1]));
param.SQbias0     = diag(deg2rad([20 20 20]));

% Get Map data
Map();
%% Get Vehicle Data
%% -- AUV data
if (param.enabled(1))
    AUV();

    data.AUV.t          = AUV_states.Time';
    data.AUV.raw.N      = length(data.AUV.raw.t);
    data.AUV.raw.X      = AUV_states.Data(:, 1:12)';
    data.AUV.raw.dnu    = AUV_states.Data(:, 13:18)';
    data.AUV.U          = AUV_U.Data';

    param.AUV.datalength = param.IMU.datalength + ...
                           param.HAP.datalength;
    
    clearvars -except data param map
    disp('Finished AUV SIM');
end

%% -- WAMV data
if (param.enabled(2))
    WAMV();

    data.WAMV.t         = WAMV_states.Time';
    data.WAMV.raw.N     = length(data.WAMV.raw.t);
    data.WAMV.raw.X     = WAMV_states.Data(:, 1:12)';
    data.WAMV.raw.dnu   = WAMV_states.Data(:, 13:18)';
    data.WAMV.U         = WAMV_U.Data';
    
    param.WAMV.datalength = param.IMU.datalength + ...
                            param.GPS.datalength  + ...
                            param.VB.datalength * map.VB.N;

    clearvars -except data param map
    disp('Finished WAMV SIM');
end
% -- QUAD data
if (param.enabled(3))
    Quadrotor();
    data.QUAD.t         = QUAD_states.Time';
    data.QUAD.raw.N     = length(data.QUAD.raw.t);
    data.QUAD.raw.X     = QUAD_states.Data(:, 1:12)';
    data.QUAD.raw.dnu   = QUAD_states.Data(:, 13:18)';
    data.QUAD.U         = QUAD_U.Data';
    
    param.QUAD.datalength = param.IMU.datalength + ...
                            param.GPS.datalength  + ...
                            param.VB.datalength*(map.VB.N+1) + ...
                            param.LPS.datalength*(map.LPS.N+1);

    clearvars -except data param map
    disp('Finished QUAD SIM');
end
assert(data.AUV.raw.N == data.WAMV.raw.N && data.AUV.raw.N == data.QUAD.raw.N, 'State trajectories must be same length');
data.ALL.N = data.AUV.raw.N;
data.t = data.AUV.t;
%% Simulate Sensor Data
data.ALL.U     = [data.AUV.U; data.WAMV.U; data.QUAD.U];


% Get all Data
data.ALL.raw = zeros(param.AUV.datalength + ...
                     param.WAMV.datalength + ...
                     param.QUAD.datalength ,param.tf*param.sensor_sample_rate);

dt = zeros(30,1);                 
for t = 1:data.ALL.N
    dtt = tic;
    data.ALL.Y(:,t)  = GetRawData([data.AUV.raw.X(:,t); data.WAMV.raw.X(:,t); data.QUAD.raw.X(:,t)], ...
                                    [data.AUV.raw.dnu(:,t); data.WAMV.raw.dnu(:,t); data.QUAD.raw.dnu(:,t)]);
   dt(1:29) = dt(2:30);
   dt(30) = toc(dtt);
   ttf = degrees2dm(sum(dt)/30*(data.ALL.N - t)/60);
   disp(['ETA data gen: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

%% KALMAN FILTER THE FUCK OUT OF SHIT
N = size(data.ALL.Y,2);     % Length of data
n = size(data.AUV.raw.X,1) + size(data.WAMV.raw.X,1) + size(data.QUAD.raw.X,1) + 9;    % Size of state vector
m = size(data.ALL.U,1);     % Size of input vector (Required to be n-n_biases)
p = size(data.ALL.Y,1);     % Size of output vector

% Create initial variances
X0 = zeros(n,1);
SQ0 = 5*blkdiag(param.AUV.SQeta, param.AUV.SQnu, param.WAMV.SQeta, param.WAMV.SQnu, param.QUAD.SQeta, param.QUAD.SQnu, ...
                param.SQbias0, param.SQbias0, param.SQbias0); 

% Initialise space for estimated means and covariances
mup = zeros(n, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)
muf = zeros(n, N);   % Filtered states

SPp  = zeros(n,n,N+1);  % Prior and Posterior Squareroot Covariances
SPf  = zeros(n,n,N+1);  % Filtered Covariances

% Set initial values
mup(:,1)    = X0;
SPp(:,:,1)   = SQ0;

U = [zeros(m,1), data.ALL.U];

for t = 1:data.ALL.N
    dtt = tic;
    
   % Measurement update
   g = @(x,u) measurementModelMonolithic(x,u);
   [muf(:,t), SPf(:,:,t)]     = UKF_MU(data.ALL.Y(:,t), mup(:,t), SPp(:,:,t), U(:,t), g);
   
   % Process Update
   f = @(x,u) processModelMonolithic(x,u);
   [mup(:,t+1), SPp(:,:,t+1)] = UKF_PU(muf(:,t), SPf(:,:,t), U(:,t+1), f);
   
   % TTF
   dt(1:29) = dt(2:30);
   dt(30) = toc(dtt);
   ttf = degrees2dm(sum(dt)/30*(data.ALL.N - t)/60);
   disp(['ETA Filtering: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

data.ALL.Xf = muf;
data.AUV.filtered = data.ALL.Xf(1:12, :);
data.WAMV.filtered = data.ALL.Xf(13:24, :);
data.QUAD.filtered = data.ALL.Xf(25:36, :);
%% PLOT ALL THE THINGS (For now)
% Plot states (True, est. mono, est. distributed)
% ETA data
f = figure(1);
columns = sum(param.enabled);
rows = 6;

% AUV ETA data
if (param.enabled(1))
    titles = {{'AUV','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
    ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
        plot(data.AUV.t, data.AUV.raw.X(row,:)*scales(row), data.AUV.t, data.AUV.filtered(row,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end

% WAMV ETA data
if (param.enabled(2))
    titles = {{'WAMV','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
    ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
        plot(data.WAMV.raw.t, data.WAMV.raw.X(row,:)*scales(row), data.WAMV.t, data.WAMV.filtered(row,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end
% --- QUAD ETA data
if (param.enabled(3))
    titles = {{'QUAD','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
    ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled));
        plot(data.QUAD.raw.t, data.QUAD.raw.X(row,:)*scales(row), data.QUAD.t, data.QUAD.filtered(row,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end
% ---------- NU data
figure(2);
% ---------- AUV NU data
if (param.enabled(1))
    titles = {{'AUV';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
        plot(data.AUV.raw.t, data.AUV.raw.X(row + 6,:)*scales(row), data.AUV.t, data.AUV.filtered(row+6,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end

end
% ---------- WAMV NU data
if (param.enabled(2))
    titles = {{'WAMV';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
        plot(data.WAMV.raw.t, data.WAMV.raw.X(row + 6,:)*scales(row), data.WAMV.t, data.WAMV.filtered(row+6,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end
% ---------- QUAD NU data
if (param.enabled(3))
    titles = {{'QUAD';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled));
        plot(data.QUAD.raw.t, data.QUAD.raw.X(row + 6,:)*scales(row), data.QUAD.t, data.QUAD.filtered(row+6,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end

% ---------- TauB data
figure(3);
% ---------- AUV TauB data
if (param.enabled(1))
    titles = {{'AUV';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
    xlabels = {'','','','','','Time (s)'};
    
    TauB = param.AUV.MRB*data.AUV.raw.dnu;
    
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
        
        plot(data.AUV.raw.t, TauB(row,:));% data.QUAD.filtered.t, data.QUAD.filtered.
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end

end
% ---------- WAMV NU data
if (param.enabled(2))
    titles = {{'WAMV';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
    xlabels = {'','','','','','Time (s)'};
    
    TauB = param.WAMV.MRB*data.WAMV.raw.dnu;
    
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
        
        plot(data.WAMV.raw.t, TauB(row,:));% data.QUAD.filtered.t, data.QUAD.filtered.
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end
% ---------- QUAD NU data
if (param.enabled(3))
    titles = {{'QUAD';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
    xlabels = {'','','','','','Time (s)'};
    
    TauB = param.QUAD.MRB*data.QUAD.raw.dnu;
    
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled));
        
        plot(data.QUAD.raw.t, TauB(row,:));% data.QUAD.filtered.t, data.QUAD.filtered.
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end

clearvars -except data param map