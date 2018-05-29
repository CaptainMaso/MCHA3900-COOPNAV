clear variables;

addpath('QUAD');
addpath('WAMV');
addpath('AUV');
addpath('Sensors');
addpath('Common');
addpath('Estimators');

%warning('off', 'MATLAB:legend:IgnoringExtraEntries');

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
param.tf = 15;

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

    param.AUV.datalength = param.IMU.datalength;% + ...
                           %param.HAP.datalength;
    
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
                            param.GPS.datalength;%  + ...
                            %param.VB.datalength * map.VB.N;

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
                            param.GPS.datalength;%  + ...
                            %param.VB.datalength*(map.VB.N+1) + ...
                            %param.LPS.datalength*(map.LPS.N+1);

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

data.AUV.Y = zeros(param.AUV.datalength, param.tf*param.sensor_sample_rate);
data.WAMV.Y = zeros(param.WAMV.datalength, param.tf*param.sensor_sample_rate);
data.QUAD.Y = zeros(param.QUAD.datalength, param.tf*param.sensor_sample_rate);
data.ALL.Y = zeros(param.AUV.datalength + ...
                     param.WAMV.datalength + ...
                     param.QUAD.datalength ,param.tf*param.sensor_sample_rate);

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
    
    % Get AUV data
    [data.AUV.Y(:,t),SR_AUV] = getRawDataAUV(data.AUV.X(:,t), data.AUV.dnu(:,t), data.WAMV.X(1:6,t), zeros(3,1));
    data.AUV.Y(:,t)     = data.AUV.Y(:,t) + SR_AUV*randn(size(SR_AUV,1),1);
    data.AUV.IMU(:,t)   = data.AUV.Y(1:9,t);
    %data.AUV.HAP(:,t)   = data.AUV.Y(10:12,t);
    
    % Get WAMV data
    [data.WAMV.Y(:,t),SR_WAMV] = getRawDataWAMV(data.WAMV.X(:,t), data.WAMV.dnu(:,t), zeros(3,1));
    data.WAMV.Y(:,t)  = data.WAMV.Y(:,t) + SR_WAMV*randn(size(SR_WAMV,1),1);
    data.WAMV.IMU(:,t)   = data.WAMV.Y(1:9,t);
    data.WAMV.GPS(:,t)   = data.WAMV.Y(10:12,t);
    %vboffset = param.VB.datalength*map.VB.N;
    %data.WAMV.VB(:,t)    = data.WAMV.Y(13:13+vboffset-1, t);
    
    % Get QUAD data
    [data.QUAD.Y(:,t),SR_QUAD] = getRawDataQUAD(data.QUAD.X(:,t), data.QUAD.dnu(:,t), data.WAMV.X(1:6,t), zeros(3,1));
    data.QUAD.Y(:,t)  = data.QUAD.Y(:,t) + SR_QUAD*randn(size(SR_QUAD,1),1);
    data.QUAD.IMU(:,t)   = data.QUAD.Y(1:9,t);
    data.QUAD.GPS(:,t)   = data.QUAD.Y(10:12,t);
    
    %vboffset    = param.VB.datalength*(map.VB.N+1);
    %lpsoffset   = param.LPS.datalength*(map.LPS.N+1);
    
    %data.QUAD.VB(:,t)    = data.QUAD.Y(13:13+vboffset-1, t);
    %data.QUAD.LPS(:,t)   = data.QUAD.Y(13+vboffset:13+vboffset+lpsoffset-1, t);
    
    % TTF    
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
    data.AUV.Est(10:12,1,t) = data.AUV.IMU(4:6,t);       % Matching gyro to angular velocities
    data.AUV.Est(13:15,1,t) = data.AUV.IMU(1:3,t) + param.g*[0,0,1]';      % Matching Accelerometer to dnu
    %data.AUV.Est(4:6,1,t)  = data.AUV.IMU(7:9          % Figure out how to
    %match Magnetometer with angles (Can only get certain angles from it?
    data.AUV.Est(4,1,t)     = atan2(-data.AUV.IMU(9,t), data.AUV.IMU(8,t));
    data.AUV.Est(5,1,t)     = atan2(-data.AUV.IMU(9,t), data.AUV.IMU(7,t));
    data.AUV.Est(6,1,t)     = atan2(data.AUV.IMU(8,t), data.AUV.IMU(7,t));
    
    % HAP estimate of AUV state (Eh can do later)
    Rwn = eulerRotation(data.WAMV.X(4:6, t));
    data.AUV.Est(1:3,1,t)     = Rwn'*data.AUV.HAP(1:3,t) + data.WAMV.GPS(1:3,t);
    
    % IMU estimate of WAMV state
    data.WAMV.Est(10:12,1,t) = data.WAMV.IMU(4:6,t);       % Matching gyro to angular velocities
    data.WAMV.Est(13:15,1,t) = data.WAMV.IMU(1:3,t) + param.g*[0,0,1]';      % Matching Accelerometer to dnu
    
    data.WAMV.Est(6,1,t)     = atan2(data.WAMV.IMU(8,t), data.WAMV.IMU(7,t));
    data.WAMV.Est(4,1,t)     = atan2(-data.WAMV.IMU(9,t), data.WAMV.IMU(8,t));
    data.WAMV.Est(5,1,t)     = atan2(-data.WAMV.IMU(9,t), data.WAMV.IMU(7,t));
    
    
    data.WAMV.Est(1,1,t)     = data.WAMV.GPS(1,t);
    data.WAMV.Est(2,1,t)     = data.WAMV.GPS(2,t);
    data.WAMV.Est(3,1,t)     = data.WAMV.GPS(3,t);
    
    % IMU estimate of QUAD state
    data.QUAD.Est(10:12,1,t) = data.QUAD.IMU(4:6,t);       % Matching gyro to angular velocities
    data.QUAD.Est(13:15,1,t) = data.QUAD.IMU(1:3,t) + param.g*[0,0,1]';      % Matching Accelerometer to dnu
    
    data.QUAD.Est(4,1,t)     = atan2(-data.QUAD.IMU(9,t), data.QUAD.IMU(8,t));
    data.QUAD.Est(5,1,t)     = atan2(-data.QUAD.IMU(9,t), data.QUAD.IMU(7,t));
    data.QUAD.Est(6,1,t)     = atan2(data.QUAD.IMU(8,t), data.QUAD.IMU(7,t));
    
    data.QUAD.Est(1,1,t)     = data.QUAD.GPS(1,t);
    data.QUAD.Est(2,1,t)     = data.QUAD.GPS(2,t);
    data.QUAD.Est(3,1,t)     = data.QUAD.GPS(3,t);
    
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
        plot(data.AUV.t, data.AUV.X(row,:)*scales(row), data.AUV.t, squeeze(data.AUV.Est(row,1,:))*scales(row), data.AUV.t, squeeze(data.AUV.Est(row,2,:))*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
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
        plot(data.WAMV.t, data.WAMV.X(row,:)*scales(row), data.WAMV.t, squeeze(data.WAMV.Est(row,1,:))*scales(row), data.WAMV.t, squeeze(data.WAMV.Est(row,2,:))*scales(row));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
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
        plot(data.QUAD.t, data.QUAD.X(row,:)*scales(row), data.QUAD.t, squeeze(data.QUAD.Est(row,1,:))*scales(row), data.QUAD.t, squeeze(data.QUAD.Est(row,2,:)*scales(row)));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end
% % ---------- NU data
figure(2);
% ---------- AUV NU data
if (param.enabled(1))
    titles = {{'AUV';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
    ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
        plot(data.AUV.t, data.AUV.X(row + 6,:)*scales(row), data.AUV.t, squeeze(data.AUV.Est(row+6,1,:)*scales(row)), data.AUV.t, squeeze(data.AUV.Est(row+6,2,:)*scales(row)));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
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
        plot(data.WAMV.t, data.WAMV.X(row + 6,:)*scales(row), data.WAMV.t, squeeze(data.WAMV.Est(row+6,1,:)*scales(row)), data.WAMV.t, squeeze(data.WAMV.Est(row+6,2,:)*scales(row)));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
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
        plot(data.QUAD.t, data.QUAD.X(row + 6,:)*scales(row), data.QUAD.t, squeeze(data.QUAD.Est(row+6,1,:)*scales(row)), data.QUAD.t, squeeze(data.QUAD.Est(row+6,2,:)*scales(row)));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end

% % ---------- TauB data
figure(3);
% ---------- AUV TauB data
if (param.enabled(1))
    titles = {{'AUV';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
    xlabels = {'','','','','','Time (s)'};
    
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
        
        plot(data.AUV.t, data.AUV.dnu(row,:), data.AUV.t, squeeze(data.AUV.Est(row+12,1,:)), data.AUV.t, squeeze(data.AUV.Est(row+12,2,:)));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end

end
% ---------- WAMV NU data
if (param.enabled(2))
    titles = {{'WAMV';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
    xlabels = {'','','','','','Time (s)'};
    
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled(1:2)));
        
        plot(data.WAMV.t, data.WAMV.dnu(row,:), data.WAMV.t, squeeze(data.WAMV.Est(row+12,1,:)), data.WAMV.t, squeeze(data.WAMV.Est(row+12,2,:)));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end
% ---------- QUAD NU data
if (param.enabled(3))
    titles = {{'QUAD';'SURGE Forces'}, 'SWAY Forces', 'HEAVE Forces', 'ROLL Forces', 'PITCH Forces', 'YAW Forces'};
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'K (Nm)', 'M (Nm)', 'N (Nm)'};
    xlabels = {'','','','','','Time (s)'};
    
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + sum(param.enabled));
        
        plot(data.QUAD.t, data.QUAD.dnu(row,:), data.QUAD.t, squeeze(data.QUAD.Est(row+12,1,:)), data.QUAD.t, squeeze(data.QUAD.Est(row+12,2,:)));
        grid on;
        t = titles(row);
        title(t{:}, 'Interpreter', 'latex');
        ylabel(ylabels(row), 'Interpreter', 'latex');
        xlabel(xlabels(row), 'Interpreter', 'latex');
        %legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
    end
end

clearvars -except data param map