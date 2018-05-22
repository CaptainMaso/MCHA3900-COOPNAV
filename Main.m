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
param.enabled = [0,1,0]; % AUV, WAMV, QUAD

% -- Universal Parameters
% --- World Parameters
param.g = 9.81;

% --- Sensor Parameters
param.sensor_sample_rate = 100;  % Sample rate (Hz)
param.IMU.samplerate = 100;

% ---- LPS parameters
param.LPS.sigma = 0.1;          % Noise on LPS data (m)
% ---- Visual Bearing (VB) parameters
param.VB.sigma      = eye(3)*0.01;      % Noise on VB data (rad)
% ---- IMU parameters
param.IMU.acc_sigma = eye(3)*0.01;      % Noise on accelerometer data (m/s^2)
param.IMU.gyro_sigma = eye(3)*0.01;     % Noise on gyro data (rad/s)
param.IMU.gyro_bias  = diag([0.4, -0.2, 0.6]);
param.IMU.magn_sigma = eye(3)*0.01;
% ---- GPS parameters
param.GPS.sigma      = eye(3)*5;       % Noise on gps data (m)

% --- Simulation Parameters
param.tf = 50;

%% Get Vehicle Data
%% -- AUV data
if (param.enabled(1))
    AUV();

    data.AUV.raw.t      = AUV_states.Time';
    data.AUV.raw.N      = length(data.AUV.raw.t);
    data.AUV.raw.X      = AUV_states.Data(:, 1:12)';
    data.AUV.raw.dnu    = AUV_states.Data(:, 13:18)';
    data.AUV.U         = AUV_U.Data';

    clearvars -except data param map
end

%% -- WAMV data
if (param.enabled(2))
    WAMV();

    data.WAMV.raw.t     = WAMV_states.Time';
    data.WAMV.raw.N     = length(data.WAMV.raw.t);
    data.WAMV.raw.X     = WAMV_states.Data(:, 1:12)';
    data.WAMV.raw.dnu   = WAMV_states.Data(:, 13:18)';
    data.WAMV.U         = WAMV_U.Data';

    clearvars -except data param map
end
% -- QUAD data
if (param.enabled(3))
    Quadrotor();
    data.QUAD.raw.t     = QUAD_states.Time';
    data.QUAD.raw.N     = length(data.QUAD.raw.t);
    data.QUAD.raw.X     = QUAD_states.Data(:, 1:12)';
    data.QUAD.raw.dnu   = QUAD_states.Data(:, 13:18)';
    data.QUAD.U         = QUAD_U.Data';

    clearvars -except data param map
end
%% Simulate Sensor Data
% Get Map data
Map();

% Get all Data
% for t = 1:param.tf*param.sensor_sample_rate
%     data.RAW.All  = GetRawData([data.AUV.X(:,t); zeros(12,1); data.QUAD.X(:,t)], ...
%                          [data.AUV.dnu(:,t); zeros(6,1); data.QUAD.dnu(:,t)]);
% end
% -- QUAD measurement simulation

% -- WAMV measurement simulation

% -- AUV measurement simulation

% -- Relative measurement simulation

%% KALMAN FILTER THE FUCK OUT OF SHIT
% Lol

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
        plot(data.AUV.raw.t, data.AUV.raw.X(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.WAMV.raw.t, data.WAMV.raw.X(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.QUAD.raw.t, data.QUAD.raw.X(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.AUV.raw.t, data.AUV.raw.X(row + 6,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.WAMV.raw.t, data.WAMV.raw.X(row + 6,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.QUAD.raw.t, data.QUAD.raw.X(row + 6,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'L (Nm)', 'M (Nm)', 'N (Nm)'};
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
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'L (Nm)', 'M (Nm)', 'N (Nm)'};
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
    ylabels = {'X (N)', 'V (N)', 'Z (N)', 'L (Nm)', 'M (Nm)', 'N (Nm)'};
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