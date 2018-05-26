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
    data.AUV.N      = length(data.AUV.t);
    data.AUV.X      = AUV_states.Data(:, 1:12)';
    data.AUV.dnu    = AUV_states.Data(:, 13:18)';
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
    data.WAMV.N     = length(data.WAMV.t);
    data.WAMV.X     = WAMV_states.Data(:, 1:12)';
    data.WAMV.dnu   = WAMV_states.Data(:, 13:18)';
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
    data.QUAD.N     = length(data.QUAD.t);
    data.QUAD.X     = QUAD_states.Data(:, 1:12)';
    data.QUAD.dnu   = QUAD_states.Data(:, 13:18)';
    data.QUAD.U         = QUAD_U.Data';
    
    param.QUAD.datalength = param.IMU.datalength + ...
                            param.GPS.datalength  + ...
                            param.VB.datalength*(map.VB.N+1) + ...
                            param.LPS.datalength*(map.LPS.N+1);

    clearvars -except data param map
    disp('Finished QUAD SIM');
end
assert(data.AUV.N == data.WAMV.N && data.AUV.N == data.QUAD.N, 'State trajectories must be same length');
data.ALL.N = data.AUV.N;
data.t = data.AUV.t;
%% Simulate Sensor Data
data.ALL.U     = [data.AUV.U; data.WAMV.U; data.QUAD.U];


% Get all Data
data.ALL.raw = zeros(param.AUV.datalength + ...
                     param.WAMV.datalength + ...
                     param.QUAD.datalength ,param.tf*param.sensor_sample_rate);

dt = zeros(30,1);                 

data.AUV.IMU = zeros(param.IMU.datalength, data.ALL.N);
data.AUV.HAP = zeros(param.HAP.datalength, data.ALL.N);

data.WAMV.IMU = zeros(param.IMU.datalength, data.ALL.N);
data.WAMV.GPS = zeros(param.GPS.datalength, data.ALL.N);
data.WAMV.VB  = zeros(param.VB.datalength * map.VB.N, data.ALL.N);

data.QUAD.IMU = zeros(param.IMU.datalength, data.ALL.N);
data.QUAD.GPS = zeros(param.GPS.datalength, data.ALL.N);
data.QUAD.VB  = zeros(param.VB.datalength * (map.VB.N+1), data.ALL.N);
data.QUAD.LPS   = zeros(param.LPS.datalength * (map.LPS.N+1), data.ALL.N);

for t = 1:data.ALL.N
    dtt = tic;
    % ----------- AUV DATA
    % Gets AUV IMU data
    [data.AUV.IMU(:,t),~] = GetIMUData([data.AUV.X(:,t);data.AUV.dnu(:,t)], 0*param.IMU.gyro_bias);
    %Y_IMU = Y_IMU + SR_IMU*randn(size(SR_IMU,1),1);

    % Gets AUV HAP data
    [data.AUV.HAP(:,t), ~] = GetHAPData([data.AUV.X(:,t);data.AUV.dnu(:,t)], data.AUV.dnu(:,t));
    %Y_HAP = Y_HAP + SR_HAP*randn(size(SR_HAP,1),1);

    % ------------- WAMV DATA
    % Gets WAMV IMU data
    [data.WAMV.IMU(:,t), ~] = GetIMUData([data.WAMV.X(:,t);data.WAMV.dnu(:,t)], 0*param.IMU.gyro_bias);
    %Y_IMU = Y_IMU + SR_IMU*randn(size(SR_IMU,1),1);

    % Gets WAMV GPS data
    [data.WAMV.GPS(:,t), ~] = GetGPSData([data.WAMV.X(:,t);data.WAMV.dnu(:,t)]);
    %Y_GPS = Y_GPS + SR_GPS*randn(size(SR_GPS,1),1);

    % Gets VB Data
    [data.WAMV.VB(:,t),~]  = GetVBData([data.WAMV.X(:,t);data.WAMV.dnu(:,t)]);
    %Y_VB = Y_VB + SR_VB*randn(size(SR_VB,1),1);
    
    % -------------------- QUAD DATA
    % Get vector from QUAD to WAMV in b (rWQb)
    Rnq     = eulerRotation(data.QUAD.X(4:6,t));
    rWQn    = data.WAMV.X(1:3,t) - data.QUAD.X(1:3,t);
    rWQq    = Rnq'*rWQn;

    % Gets IMU data
    [data.QUAD.IMU(:,t), ~] = GetIMUData([data.QUAD.X(:,t);data.QUAD.dnu(:,t)],0*param.IMU.gyro_bias);
    %Y_IMU = Y_IMU + SR_IMU*randn(size(SR_IMU,1),1);

    % Gets GPS data
    [data.QUAD.GPS(:,t), ~] = GetGPSData([data.QUAD.X(:,t);data.QUAD.dnu(:,t)]);
    %Y_GPS = Y_GPS + SR_GPS*randn(size(SR_GPS,1),1);

    % Gets VB Data
    [Y_VB,~]    = GetVBData([data.QUAD.X(:,t);data.QUAD.dnu(:,t)]);

    if (norm(rWQq) == 0)
       vb_wamv = [0;0;0];
    else
       vb_wamv = rWQq/norm(rWQq);
    end

    %SR_VB   = blkdiag(SR_VB, param.VB.sigma);
    data.QUAD.VB(:,t)    = [Y_VB;vb_wamv];% + SR_VB*randn(size(SR_VB,1),1);  % Adds normalised bearing vector from quad to wamv

    % Gets LPS Data
    [Y_LPS,~] = GetLPSData([data.QUAD.X(:,t);data.QUAD.dnu(:,t)]);

    lps_wamv = norm(rWQn);

    %SR_LPS  = blkdiag(SR_LPS, param.LPS.sigma);
    data.QUAD.LPS(:,t)   = [Y_LPS; lps_wamv];% + SR_LPS*randn(size(SR_LPS,1),1);     % Adds distance from quad to wamv to LPS data
    
    dt(1:29) = dt(2:30);
    dt(30) = toc(dtt);
    ttf = degrees2dm(sum(dt)/30*(data.ALL.N - t)/60);
    disp(['ETA data gen: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end
clearvars -except data param map
%% Create data in estimated format
data.AUV.Est = nan(18,2,data.ALL.N);
data.WAMV.Est = nan(18,3,data.ALL.N);
data.QUAD.Est = nan(18,4,data.ALL.N);

for t = 1:data.ALL.N
    % IMU Estimate of AUV state
    data.AUV.Est(9:12,1,t) = data.AUV.IMU(4:6,t);       % Matching gyro to angular velocities
    data.AUV.Est(13:15,1,t) = data.AUV.IMU(1:3,t);      % Matching Accelerometer to dnu
    %data.AUV.Est(4:6,1,t)  = data.AUV.IMU(7:9          % Figure out how to
    %match Magnetometer with angles (Can only get certain angles from it?
    %Phi when Psi = 0; Theta when Psi = pi/2
    
    % HAP estimate of AUV state (Eh can do later)
    
    
    % IMU estimate of WAMV state
    data.WAMV.Est(9:12,1,t) = data.WAMV.IMU(4:6,t);       % Matching gyro to angular velocities
    data.WAMV.Est(13:15,1,t) = data.WAMV.IMU(1:3,t);      % Matching Accelerometer to dnu
    
    % IMU estimate of QUAD state
    data.QUAD.Est(9:12,1,t) = data.QUAD.IMU(4:6,t);       % Matching gyro to angular velocities
    data.QUAD.Est(13:15,1,t) = data.QUAD.IMU(1:3,t);      % Matching Accelerometer to dnu
    
end

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
        plot(data.AUV.t, data.AUV.X(row,:)*scales(row), data.AUV.t, data.AUV.Xf(row,:)*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end