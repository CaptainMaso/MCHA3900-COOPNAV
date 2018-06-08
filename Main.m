clear variables data;

addpath('QUAD');
addpath('WAMV');
addpath('AUV');
addpath('Sensors');
addpath('Common');
addpath('Estimators');

warning('off', 'MATLAB:legend:IgnoringExtraEntries');

global param map data
%% Parameter Initialisation
% Get Map data
Map();
param.enabled = [1,1,1]; % AUV, WAMV, QUAD

% -- Universal Parameters
% --- World Parameters
param.g = 9.81;

% --- Sensor Parameters
param.sensor_sample_rate = 200;  % Sample rate (Hz)

% ---- LPS parameters
param.LPS.sigma = 1e-4;          % Noise on LPS data (m)
param.LPS.datalength = 1;

% ---- Visual Bearing (VB) parameters
param.VB.sigma      = eye(3)*1e-4;      % Noise on VB data (rad)
param.VB.datalength = 3;

% ---- IMU parameters
param.IMU.acc_sigma = eye(3)*1e-4;      % Noise on accelerometer data (m/s^2)
param.IMU.gyro_sigma = eye(3)*1e-4;     % Noise on gyro data (rad/s)
param.IMU.gyro_bias  = deg2rad([30, -40, 60])';
param.IMU.magn_sigma = eye(3)*1e-3;
param.IMU.datalength = 9;

% ---- GPS parameters
param.GPS.sigma      = eye(3)*1e-2;       % Noise on gps data (m)
param.GPS.datalength = 3;

% ---- HAP parameters
param.HAP.sigma      = eye(3)*1e-4;       % Noise on HAP data (m)
param.HAP.datalength = 3;

% --- Simulation Parameters
param.tf = 10;

% --- Vehicle Uncertainties
param.AUV.SQeta = diag([1e-3 1e-3 1e-3, deg2rad([20 20 20])])./param.sensor_sample_rate;
param.AUV.SQnu  = diag([1e-2 1e-2 1e-2,  deg2rad([40 40 40])])./param.sensor_sample_rate;

param.WAMV.SQeta = diag([1e-2 1e-2 1e-1, deg2rad([60 60 5])])./param.sensor_sample_rate;
param.WAMV.SQnu  = diag([5e-1 5e-1 5e0  deg2rad([1e1 1e1 25])])./param.sensor_sample_rate;

param.QUAD.SQeta = diag([1e-2 1e-2 1e-2, deg2rad([1 1 1])])./param.sensor_sample_rate;
param.QUAD.SQnu  = diag([2e1 2e1 2e2, deg2rad([5 5 5])])./param.sensor_sample_rate;

param.IMU.SQbias     = diag(deg2rad([0.01 0.01 0.01]));
param.IMU.SQbias0     = diag(deg2rad([1e3 1e3 1e3]));

% --- Vehicle data parameters
param.AUV.datalength  = param.IMU.datalength + ...
                        param.HAP.datalength;

param.WAMV.datalength = param.IMU.datalength + ...
                        param.GPS.datalength  + ...
                        param.VB.datalength * map.VB.N;
                        
param.QUAD.datalength = param.IMU.datalength + ...
                        param.GPS.datalength + ...
                        param.VB.datalength*(map.VB.N+1) + ...
                        param.LPS.datalength*(map.LPS.N+1);

%% Get Vehicle Data
%% -- AUV data
if (param.enabled(1))
    AUV();

    data.AUV.t          = AUV_states.Time';
    data.AUV.N          = length(data.AUV.t);
    data.AUV.X          = AUV_states.Data(:, 1:12)';
    data.AUV.dnu        = AUV_states.Data(:, 13:18)';
    data.AUV.U          = AUV_U.Data';
    data.AUV.U         = [zeros(size(data.AUV.U,1),1), data.AUV.U];
    
    clearvars -except data param map
    disp('Finished AUV SIM');
end

%% -- WAMV data
if (param.enabled(2))
    WAMV();

    data.WAMV.t         = WAMV_states.Time';
    data.WAMV.N         = length(data.WAMV.t);
    data.WAMV.X         = WAMV_states.Data(:, 1:12)';
    data.WAMV.dnu       = WAMV_states.Data(:, 13:18)';
    data.WAMV.U         = WAMV_U.Data';
    data.WAMV.U         = [zeros(size(data.WAMV.U,1),1), data.WAMV.U];

    clearvars -except data param map
    disp('Finished WAMV SIM');
end
% -- QUAD data
if (param.enabled(3))
    Quadrotor();
    data.QUAD.t         = QUAD_states.Time';
    data.QUAD.N         = length(data.QUAD.t);
    data.QUAD.X         = QUAD_states.Data(:, 1:12)';
    data.QUAD.dnu       = QUAD_states.Data(:, 13:18)';
    data.QUAD.U         = QUAD_U.Data';
    data.QUAD.U         = [zeros(size(data.QUAD.U,1),1), data.QUAD.U];
    
    clearvars -except data param map
    disp('Finished QUAD SIM');
end
assert(data.AUV.N == data.WAMV.N && data.AUV.N == data.QUAD.N, 'State trajectories must be same length');
data.MONO.N = data.AUV.N;
data.t = data.AUV.t;

%% Simulate Sensor Data

% Get all Data
data.AUV.Y  = zeros(param.AUV.datalength,  param.tf*param.sensor_sample_rate);
data.WAMV.Y = zeros(param.WAMV.datalength, param.tf*param.sensor_sample_rate);
data.QUAD.Y = zeros(param.QUAD.datalength, param.tf*param.sensor_sample_rate);

dt = zeros(30,1);                 
for t = 1:data.MONO.N
    dtt = tic;
    
    % Get AUV data
    [data.AUV.Y(:,t),SR_AUV] = getRawDataAUV(data.AUV.X(:,t), data.AUV.dnu(:,t), data.WAMV.X(1:6,t), param.IMU.gyro_bias);
    data.AUV.Y(:,t)     = data.AUV.Y(:,t) + SR_AUV*randn(size(SR_AUV,1),1);
%         data.AUV.IMU(:,t)   = data.AUV.Y(1:9,t);
%         data.AUV.HAP(:,t)   = data.AUV.Y(10:12,t);

    % Get WAMV data
    [data.WAMV.Y(:,t),SR_WAMV] = getRawDataWAMV(data.WAMV.X(:,t), data.WAMV.dnu(:,t), param.IMU.gyro_bias);
    data.WAMV.Y(:,t)  = data.WAMV.Y(:,t) + SR_WAMV*randn(size(SR_WAMV,1),1);
%         data.WAMV.IMU(:,t)   = data.WAMV.Y(1:9,t);
%         data.WAMV.GPS(:,t)   = data.WAMV.Y(10:12,t);
%         vboffset = param.VB.datalength*map.VB.N;
%         data.WAMV.VB(:,t)    = data.WAMV.Y(13:13+vboffset-1, t);

    % Get QUAD data
    [data.QUAD.Y(:,t),SR_QUAD] = getRawDataQUAD(data.QUAD.X(:,t), data.QUAD.dnu(:,t), data.WAMV.X(1:6,t), param.IMU.gyro_bias);
    data.QUAD.Y(:,t)  = data.QUAD.Y(:,t) + SR_QUAD*randn(size(SR_QUAD,1),1);
%         data.QUAD.IMU(:,t)   = data.QUAD.Y(1:9,t);
%         data.QUAD.GPS(:,t)   = data.QUAD.Y(1:3,t);
% 
%         vboffset    = param.VB.datalength*(map.VB.N+1);
%         lpsoffset   = param.LPS.datalength*(map.LPS.N+1);

    %data.QUAD.VB(:,t)    = data.QUAD.Y(13:13+vboffset-1, t);
    %data.QUAD.LPS(:,t)   = data.QUAD.Y(13+vboffset:13+vboffset+lpsoffset-1, t);
    
    % TTF
    dt(1:29) = dt(2:30);
    dt(30) = toc(dtt);
    ttf = degrees2dm(sum(dt)/sum(dt ~= 0)*(data.MONO.N - t)/60);
    disp(['ETA data gen: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

%% Estimation
mono_sub_switch = 2;            % 0 = Sub 1 = Mono;  2 = both/compare

if mono_sub_switch ~= 0 && mono_sub_switch ~= 1 && mono_sub_switch ~= 2
    error('mono_sub_switch must be 0, 1, or 2');
end

N = size(data.QUAD.Y,2);     % Length of data
n = size(data.AUV.X,1) + size(data.WAMV.X,1) + size(data.QUAD.X,1) + 9;
m = size(data.AUV.Y,1) + size(data.WAMV.Y,1) + size(data.QUAD.Y,1);
p = size(data.AUV.U,1) + size(data.WAMV.U,1) + size(data.QUAD.U,1);

data.MONO.Y = [data.AUV.Y;data.WAMV.Y;data.QUAD.Y];
data.MONO.U = [data.AUV.U;data.WAMV.U;data.QUAD.U];

% Initialise space for estimated means and covariances
data.AUV.Xf  = zeros(21, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)
data.WAMV.Xf = zeros(15, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)
data.QUAD.Xf = zeros(21, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)

data.AUV.SPf   = zeros(21,21,N+1);  % Prior and Posterior Squareroot Covariances
data.WAMV.SPf  = zeros(15,15,N+1);  % Prior and Posterior Squareroot Covariances
data.QUAD.SPf  = zeros(21,21,N+1);  % Prior and Posterior Squareroot Covariances

data.MONO.Xf    = zeros(n, N+1); % Mean Filtered states
data.MONO.SPf   = zeros(n,n,N+1);  % Prior and Posterior Squareroot Covariances

% Set initial values
data.MONO.SPf(:,:,1)    = 1e-3*blkdiag(param.AUV.SQeta, param.AUV.SQnu, param.IMU.SQbias0, ...
                                    param.WAMV.SQeta, param.WAMV.SQnu, param.IMU.SQbias0, ...
                                    param.QUAD.SQeta, param.QUAD.SQnu, param.IMU.SQbias0);
                                
data.AUV.SPf(:,:,1)      = 1e-3*blkdiag(param.AUV.SQeta, param.AUV.SQnu, ...
                                        param.IMU.SQbias0, param.WAMV.SQeta);
data.WAMV.SPf(:,:,1)     = 1e-3*blkdiag(param.WAMV.SQeta, param.WAMV.SQnu, ...
                                        param.IMU.SQbias0);
data.QUAD.SPf(:,:,1)     = 1e-3*blkdiag(param.QUAD.SQeta, param.QUAD.SQnu, ...
                                        param.IMU.SQbias0, param.WAMV.SQeta);


for t = 1:data.MONO.N
    dtt = tic;
    if mono_sub_switch ~= 0 && mono_sub_switch ~= 1 && mono_sub_switch ~= 2
        error('shits fucked');
    end

    if mono_sub_switch == 0 || mono_sub_switch == 2    % Sub estimator stuff 
        Xmono_pri  = [data.AUV.Xf(1:15,t);
                      data.WAMV.Xf(1:15,t);
                      data.QUAD.Xf(1:15,t)];
                  
        SPmono_pri = blkdiag(data.AUV.SPf(1:15,1:15,t), ...
                             data.WAMV.SPf(1:15,1:15,t), ...
                             data.QUAD.SPf(1:15,1:15,t));
        
        % AUV Estimator
        g_auv = @(x,u) mm_auv(x(1:12), u, x(13:15), x(16:21));      % AUV States: [AUV_eta; AUV_nu; AUV_gyrobias; WAMV_eta];
        [data.AUV.Xf(:,t), data.AUV.SPf(:,:,t)] = UKF_MU(data.AUV.Y(:,t), data.AUV.Xf(:,t), data.AUV.SPf(:,:,t), data.AUV.U(:,t), g_auv);
        
        % WAMV Estimator
        g_wamv = @(x,u) mm_wamv(x(1:12), u, x(13:15));              % WAMV States: [WAMV_eta; WAMV_nu; WAMV_gyrobias];
        [data.WAMV.Xf(:,t), data.WAMV.SPf(:,:,t)] = UKF_MU(data.WAMV.Y(:,t), data.WAMV.Xf(:,t), data.WAMV.SPf(:,:,t), data.WAMV.U(:,t), g_wamv);
        
        % QUAD Estimator
        g_quad = @(x,u) mm_quad(x(1:12), u, x(13:15), x(16:21));    % QUAD States: [QUAD_eta; QUAD_nu; QUAD_gyrobias; WAMV_eta];
        [data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t)] = UKF_MU(data.QUAD.Y(:,t), data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t), data.QUAD.U(:,t), g_quad);
        
        % Reassemble into full state for merge (This step can hapen
        % individually on each vehicle)
        % AUV Reassembly
        Xfm_auv = Xmono_pri;                            SPfm_auv = SPmono_pri;
        Xfm_auv([1:15, 16:21]) = data.AUV.Xf(:,t);      SPfm_auv([1:15,16:21],[1:15,16:21]) = data.AUV.SPf(:,:,t);
        
        % WAMV Reassembly
        Xfm_wamv = Xmono_pri;                    SPfm_wamv = SPmono_pri;
        Xfm_wamv(16:30) = data.WAMV.Xf(:,t);     SPfm_wamv(16:30,16:30) = data.WAMV.SPf(:,:,t);
        
        % QUAD Reassembly
        Xfm_quad = Xmono_pri;                            SPfm_quad = SPmono_pri;
        Xfm_quad([31:45, 16:21]) = data.QUAD.Xf(:,t);    SPfm_quad([31:45,16:21],[31:45,16:21]) = data.QUAD.SPf(:,:,t);
        
        % THIS IS WHERE "DISTRIBUTION OF THE STATES" can occur
        
        % Merge AUV/WAMV/QUAD Estimators (Can happen individually on each
        % vehicle)
        SPf_mono = inv(inv(SPfm_auv) + inv(SPfm_wamv) + inv(SPfm_quad) - 2*inv(SPmono_pri));
        Xf_mono = SPf_mono*(SPfm_auv\Xfm_auv + SPfm_wamv\Xfm_wamv + SPfm_quad\Xfm_quad - 2*(SPmono_pri\Xmono_pri));

        % UKF_PU
        f = @(x,u) pm_mono(x,u);
        [Xf_mono, SPf_mono] = UKF_PU(Xf_mono, SPf_mono, [data.AUV.U(:,t);data.WAMV.U(:,t);data.QUAD.U(:,t)], f);

        % Full state disassembly
        data.AUV.Xf(:,t+1)  = Xf_mono([1:15, 16:21]);    data.AUV. SPf(:,:,t+1) = SPf_mono([1:15,16:21],[1:15,16:21]);
        data.WAMV.Xf(:,t+1) = Xf_mono(16:30);            data.WAMV.SPf(:,:,t+1) = SPf_mono(16:30,16:30);
        data.QUAD.Xf(:,t+1) = Xf_mono([31:45, 16:21]);   data.QUAD.SPf(:,:,t+1) = SPf_mono([31:45,16:21],[31:45,16:21]);
    end

    if mono_sub_switch == 1 || mono_sub_switch == 2 % Mono stuff   
        %     Measurement update - Monolithic
        g = @(x,u) mm_mono(x, u);
        [data.MONO.Xf(:,t), data.MONO.SPf(:,:,t)]     = UKF_MU(data.MONO.Y(:,t), data.MONO.Xf(:,t), data.MONO.SPf(:,:,t), data.MONO.U(:,t), g);
    
        %     Process Update - Monolithic
        f = @(x,u) pm_mono(x,u);
        [data.MONO.Xf(:,t+1), data.MONO.SPf(:,:,t+1)] = UKF_PU(data.MONO.Xf(:,t), data.MONO.SPf(:,:,t), data.MONO.U(:,t+1), f);
    end
    
    % TTF
    dt(1:29) = dt(2:30);
    dt(30) = toc(dtt);
    ttf = degrees2dm(sum(dt)/sum(dt ~= 0)*(data.MONO.N - t)/60);
    disp(['ETA Filtering: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

%% Resize and separate filtered data
data.MONO.Xf  = data.MONO.Xf(:,1:end-1);
data.AUV.Xf   = data.AUV.Xf(:,1:end-1);
data.WAMV.Xf  = data.WAMV.Xf(:,1:end-1);
data.QUAD.Xf  = data.QUAD.Xf(:,1:end-1);

%%

PlotData(data, mono_sub_switch);

clearvars -except data param map 