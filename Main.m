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

% Get Map data
Map();
%% Get Vehicle Data
%% -- AUV data
if (param.enabled(1))
    AUV();

    data.AUV.raw.t      = AUV_states.Time';
    data.AUV.raw.N      = length(data.AUV.raw.t);
    data.AUV.raw.X      = AUV_states.Data(:, 1:12)';
    data.AUV.raw.dnu    = AUV_states.Data(:, 13:18)';
    data.AUV.U          = AUV_U.Data';

    param.AUV.datalength = param.IMU.datalength + ...
                           param.HAP.datalength;
    
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
    
    param.WAMV.datalength = param.IMU.datalength + ...
                            param.GPS.datalength  + ...
                            param.VB.datalength * map.VB.N;

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
    
    param.QUAD.datalength = param.IMU.datalength + ...
                            param.GPS.datalength  + ...
                            param.VB.datalength*(map.VB.N+1) + ...
                            param.LPS.datalength*(map.LPS.N+1);

    clearvars -except data param map
end
%% Simulate Sensor Data


% Get all Data
data.ALL.raw = zeros(param.AUV.datalength + ...
                     param.WAMV.datalength + ...
                     param.QUAD.datalength ,param.tf*param.sensor_sample_rate);
for t = 1:param.tf*param.sensor_sample_rate+1
    data.ALL.raw(:,t)  = GetRawData([data.AUV.raw.X(:,t); data.WAMV.raw.X(:,t); data.QUAD.raw.X(:,t)], ...
                                    [data.AUV.raw.dnu(:,t); data.WAMV.raw.dnu(:,t); data.QUAD.raw.dnu(:,t)]);
end

%% KALMAN FILTER THE FUCK OUT OF SHIT
N = data.AUV.raw.N;
U = [data.AUV.U;data.WAMV.U;data.QUAD.U;zeros(5,length(data.QUAD.U))];
U = [zeros(18,1),U];
Y = data.ALL.raw;

n = size(data.AUV.raw.X,1) + size(data.WAMV.raw.X,1)+ size(data.QUAD.raw.X,1) + 9; % Length of states
m = size(U,1);      % Length of inputs
p = size(Y,1);      % Length of outputs

mup = zeros(n,1);
SPp = diag(ones(n,1));

muf_mono = zeros(n,N);
for t = 1:N
   % AUV MU
    u = U(1:6,t);
    f =  @(x,u) measurementModelAUV(x,u);
    jointFunc               = @(x,u) augmentIdentityAdapter(f, x, u);
    [muxy,Syx]              = unscentedTransform(mup, SPp, jointFunc, u);
    [muf_auv, SPf_auv]       = conditionGaussianOnMarginal(muxy, Syx, Y(1:param.AUV.datalength,t));       
    offset = param.AUV.datalength;
   % WAMV MU
    u = U(7:12,t);
    f =  @(x,u) measurementModelWAMV(x,u);
    jointFunc               = @(x,u) augmentIdentityAdapter(f, x, u);
    [muxy,Syx]              = unscentedTransform(mup, SPp, jointFunc, u);
    [muf_wamv, SPf_wamv]       = conditionGaussianOnMarginal(muxy, Syx, Y(param.AUV.datalength+1:param.AUV.datalength+param.WAMV.datalength,t));
    offset = offset + param.WAMV.datalength;
   %QUAD MU
    u = U(13:18,t);
    f =  @(x,u) measurementModelQUAD(x,u);
    jointFunc               = @(x,u) augmentIdentityAdapter(f, x, u);
    [muxy,Syx]              = unscentedTransform(mup, SPp, jointFunc, u);
    [muf_quad, SPf_quad]       = conditionGaussianOnMarginal(muxy, Syx, Y(param.AUV.datalength+param.WAMV.datalength+1:param.AUV.datalength+param.WAMV.datalength+ param.QUAD.datalength,t));
    
    SPf_mono = inv(inv(SPf_auv) + inv(SPf_wamv) + inv(SPf_quad) - 2*inv(SPp));
    muf_mono(:,t) = SPf_mono*(SPf_auv\muf_auv + SPf_wamv\muf_wamv + SPf_quad\muf_quad - 2*inv(SPp)*mup);
   
    % UKF_PU   
    u = U(:,t+1);
    processFunc  = @(x,u) processModelMonolithic(x, u);
    [mu_next, S_next] = unscentedTransform(muf_mono(:,t), SPf_mono, processFunc, u);
   
end

%% PLOT ALL THE THINGS (For now)
% Plot states (True, est. mono, est. distributed)
% ETA data
f = figure(1);
columns = sum(param.enabled);
rows = 6;
data.AUV.post_data = muf_mono(1:12,:);
data.WAMV.post_data = muf_mono(13:24,:);
data.QUAD.post_data = muf_mono(25:36,:);

% AUV ETA data
if (param.enabled(1))
    titles = {{'AUV','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
    ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
    xlabels = {'','','','','','Time (s)'};
    scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
    for row=1:rows
        subplot(rows,columns,(row-1)*columns + 1);
        plot(data.AUV.raw.t, data.AUV.raw.X(row,:)*scales(row),data.AUV.raw.t, data.AUV.post_data(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.WAMV.raw.t, data.WAMV.raw.X(row,:)*scales(row),data.WAMV.raw.t, data.WAMV.post_data(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.QUAD.raw.t, data.QUAD.raw.X(row,:)*scales(row),data.QUAD.raw.t, data.QUAD.post_data(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.AUV.raw.t, data.AUV.raw.X(row + 6,:)*scales(row),data.AUV.raw.t, data.AUV.post_data(row + 6,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.WAMV.raw.t, data.WAMV.raw.X(row + 6,:)*scales(row),data.WAMV.raw.t, data.WAMV.post_data(row + 6,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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
        plot(data.QUAD.raw.t, data.QUAD.raw.X(row + 6,:)*scales(row),data.QUAD.raw.t, data.QUAD.post_data(row + 6,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
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