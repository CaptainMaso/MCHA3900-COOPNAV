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
param.sensor_sample_rate = 100;  % Sample rate (Hz)

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
param.GPS.sigma      = eye(3)*1e-1;       % Noise on gps data (m)
param.GPS.datalength = 3;

% ---- HAP parameters
param.HAP.sigma      = eye(3)*1e-4;       % Noise on HAP data (m)
param.HAP.datalength = 3;

% --- Simulation Parameters
param.tf = 5;

% --- Vehicle Uncertainties
param.AUV.SQeta = diag([1e-2 1e-2 1e-2, deg2rad([20 20 20])])./param.sensor_sample_rate;
param.AUV.SQnu  = diag([1e0 1e0 1e0,  deg2rad([40 40 40])])./param.sensor_sample_rate;

param.WAMV.SQeta = diag([1e-2 1e-2 2e-2, deg2rad([15 15 5])])./param.sensor_sample_rate;
param.WAMV.SQnu  = diag([5e-1 5e-1 5e0  deg2rad([3e3 3e3 25])])./param.sensor_sample_rate;

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
param.maxDelay = 4;
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
DelayedMeasurements();

%% KALMAN FILTER THE FUCK OUT OF SHIT
N = size(data.QUAD.Y,2);     % Length of data
n = size(data.AUV.X,1) + size(data.WAMV.X,1) + size(data.QUAD.X,1) + 9;
m = size(data.AUV.Y,1) + size(data.WAMV.Y,1) + size(data.QUAD.Y,1);
p = size(data.AUV.U,1) + size(data.WAMV.U,1) + size(data.QUAD.U,1);

% Initialise space for estimated means and covariances
data.AUV.Xf  = zeros(21*param.maxDelay, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)
data.WAMV.Xf = zeros(15*param.maxDelay, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)
data.QUAD.Xf = zeros(21*param.maxDelay, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)

data.AUV.SPf   = zeros(21*param.maxDelay,21*param.maxDelay,N+1);  % Prior and Posterior Squareroot Covariances
data.WAMV.SPf  = zeros(15*param.maxDelay,15*param.maxDelay,N+1);  % Prior and Posterior Squareroot Covariances
data.QUAD.SPf  = zeros(21*param.maxDelay,21*param.maxDelay,N+1);  % Prior and Posterior Squareroot Covariances

% Set initial values
for ii = 0:param.maxDelay
    
    data.AUV.SPf(1:21,1:21,1)      = 1e-3*blkdiag(param.AUV.SQeta, param.AUV.SQnu, ...
                                            param.IMU.SQbias0, param.WAMV.SQeta);
    data.WAMV.SPf(1:15,1:15,1)     = 1e-3*blkdiag(param.WAMV.SQeta, param.WAMV.SQnu, ...
                                            param.IMU.SQbias0);
    data.QUAD.SPf(1:21,1:21,1)     = 1e-3*blkdiag(param.QUAD.SQeta, param.QUAD.SQnu, ...
                                            param.IMU.SQbias0, param.WAMV.SQeta);
end

for t = 1:data.MONO.N
    dtt = tic;
    
    Xmono_pri  = nan(n*param.maxDelay,1);
    SPmono_pri = zeros(n*param.maxDelay,n*param.maxDelay);
    
    for ii = 0:param.maxDelay-1
        Xmono_pri((1:45) + ii*45)  = [data.AUV.Xf((1:15) + ii*21,t);
                                      data.WAMV.Xf((1:15) + ii*15,t);
                                      data.QUAD.Xf((1:15) + ii*21,t)];

        SPmono_pri((1:45) + ii*45, (1:45) + ii*45) = blkdiag(data.AUV.SPf((1:15) + ii*21,(1:15) + ii*21,t), ...
                                                             data.WAMV.SPf((1:15) + ii*15,(1:15) + ii*15,t), ...
                                                             data.QUAD.SPf((1:15) + ii*21,(1:15) + ii*21,t));
    end
    
    for ii = 1:cellCounter(t)
       k_n = t - Ydelay{ii,t}.timeStamp;
       if (k_n < param.maxDelay)
           switch (Ydelay{ii,t}.type)
               % AUV Measurement Corrections
               case "AUV IMU"
                    asv = (1:21) + (k_n*21); % Active State Vector
                    g_auv_imu = @(x,u) mm_AUV_IMU(x((1:12) + k_n*21), u, x((13:15) + k_n*21), x((16:21) + k_n*21));            % AUV States: [AUV_eta; AUV_nu; AUV_gyrobias; WAMV_eta];
                    [data.AUV.Xf(:,t), data.AUV.SPf(:,:,t)]   = UKF_MU(Ydelay{ii,t}.data, data.AUV.Xf(:,t), ...
                                                                             data.AUV.SPf(:,:,t), data.AUV.U(:,t-k_n), g_auv_imu);                                       
               case "AUV HAP"
                    g_auv_hap = @(x,u) mm_AUV_HAP(x((1:12) + k_n*21), u, x((13:15) + k_n*21), x((16:21) + k_n*21));               % AUV States: [AUV_eta; AUV_nu; AUV_gyrobias; WAMV_eta];
                    [data.AUV.Xf(:,t), data.AUV.SPf(:,:,t)]   = UKF_MU(Ydelay{ii,t}.data, data.AUV.Xf(:,t), ...
                                                                             data.AUV.SPf(:,:,t), data.AUV.U(:,t-k_n), g_auv_hap);
               % WAMV Measurement Corrections
               case "WAMV IMU"
                   g_wamv_imu = @(x,u) mm_WAMV_IMU(x((1:12) + k_n*15), u, x((13:15) + k_n*15));               % WAMV States: [WAMV_eta; WAMV_nu; WAMV_gyrobias];
                   [data.WAMV.Xf(:,t), data.WAMV.SPf(:,:,t)]  = UKF_MU(Ydelay{ii,t}.data, data.WAMV.Xf(:,t), ...
                                                                       data.WAMV.SPf(:,:,t), data.WAMV.U(:,t-k_n), g_wamv_imu);
               case "WAMV GPS"
                   g_wamv_gps = @(x,u) mm_WAMV_GPS(x((1:12) + k_n*15), u, x((13:15) + k_n*15));               % WAMV States: [WAMV_eta; WAMV_nu; WAMV_gyrobias];
                   [data.WAMV.Xf(:,t), data.WAMV.SPf(:,:,t)]  = UKF_MU(Ydelay{ii,t}.data, data.WAMV.Xf(:,t), ...
                                                                       data.WAMV.SPf(:,:,t), data.WAMV.U(:,t-k_n), g_wamv_gps);
               case "WAMV VB"
                    g_wamv_vb = @(x,u) mm_WAMV_VB(x((1:12) + k_n*15), u, x((13:15) + k_n*15));               % WAMV States: [WAMV_eta; WAMV_nu; WAMV_gyrobias];
                    [data.WAMV.Xf(:,t), data.WAMV.SPf(:,:,t)]  = UKF_MU(Ydelay{ii,t}.data, data.WAMV.Xf(:,t), ...
                                                                             data.WAMV.SPf(:,:,t), data.WAMV.U(:,t-k_n), g_wamv_vb);
               % QUAD Measurement Corrections
               case "QUAD IMU"
                    g_quad_imu = @(x,u) mm_QUAD_IMU(x((1:12) + k_n*21), u, x((13:15) + k_n*21), x((16:21) + k_n*21));    % QUAD States: [QUAD_eta; QUAD_nu; QUAD_gyrobias; WAMV_eta];
                    [data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t)] = UKF_MU(Ydelay{ii,t}.data, data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t), data.QUAD.U(:,t-k_n), g_quad_imu);

               case "QUAD GPS"
                    g_quad_gps = @(x,u) mm_QUAD_GPS(x((1:12) + k_n*21), u, x((13:15) + k_n*21), x((16:21) + k_n*21));    % QUAD States: [QUAD_eta; QUAD_nu; QUAD_gyrobias; WAMV_eta];
                    [data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t)] = UKF_MU(Ydelay{ii,t}.data, data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t), data.QUAD.U(:,t-k_n), g_quad_gps);

               case "QUAD VB"
                   g_quad_vb = @(x,u)   mm_QUAD_VB(x((1:12) + k_n*21), u, x((13:15) + k_n*21), x((16:21) + k_n*21));    % QUAD States: [QUAD_eta; QUAD_nu; QUAD_gyrobias; WAMV_eta];
                   [data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t)] = UKF_MU(Ydelay{ii,t}.data, data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t), data.QUAD.U(:,t-k_n), g_quad_vb);

               case "QUAD LPS"
                   g_quad_lps = @(x,u)  mm_QUAD_LPS(x((1:12) + k_n*21), u, x((13:15) + k_n*21), x((16:21) + k_n*21));    % QUAD States: [QUAD_eta; QUAD_nu; QUAD_gyrobias; WAMV_eta];
                   [data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t)] = UKF_MU(Ydelay{ii,t}.data, data.QUAD.Xf(:,t), data.QUAD.SPf(:,:,t), data.QUAD.U(:,t-k_n), g_quad_lps);

               otherwise
                   error('Who the fuck wrote this code');
           end
       end
    end
    
    Xf_mono     = Xmono_pri;
    SPf_mono    = SPmono_pri;
    
    for ii = 0:param.maxDelay-1
        Xfm_auv  = Xmono_pri((1:45) + ii*45);           SPfm_auv  = SPmono_pri((1:45) + ii*45, (1:45) + ii*45);
        Xfm_wamv = Xmono_pri((1:45) + ii*45);           SPfm_wamv = SPmono_pri((1:45) + ii*45, (1:45) + ii*45);
        Xfm_quad = Xmono_pri((1:45) + ii*45);           SPfm_quad = SPmono_pri((1:45) + ii*45, (1:45) + ii*45);
        
        if (rank(SPfm_auv) == size(SPfm_auv,1) && rank(SPfm_wamv) == size(SPfm_wamv,1) && rank(SPfm_quad) == size(SPfm_quad,1))
            % Reassemble into full state for merge (This step can hapen
            % individually on each vehicle)
            % AUV Reassembly
            Xfm_auv([1:15, 16:21])              = data.AUV.Xf((1:21) + ii*21,t);
            SPfm_auv([1:15,16:21],[1:15,16:21]) = data.AUV.SPf([1:15,16:21] + ii*21,[1:15,16:21] + ii*21,t);

            % WAMV Reassembly
            Xfm_wamv(16:30)         = data.WAMV.Xf((1:15) + ii*15,t);     
            SPfm_wamv(16:30,16:30)  = data.WAMV.SPf((1:15) + ii*15,(1:15) + ii*15,t);

            % QUAD Reassembly
            Xfm_quad([31:45, 16:21]) = data.QUAD.Xf((1:21) + ii*21,t);
            SPfm_quad([31:45,16:21],[31:45,16:21]) = data.QUAD.SPf((1:21) + ii*21, (1:21) + ii*21, t);

            % THIS IS WHERE "DISTRIBUTION OF THE STATES" can occur
            % Merge AUV/WAMV/QUAD Estimators (Can happen individually on each
            % vehicle)
            SPf_mono((1:45) + ii*45, (1:45) + ii*45)    = inv(inv(SPfm_auv) + inv(SPfm_wamv) + inv(SPfm_quad)...
                                                              - 2*inv(SPmono_pri((1:45) + ii*45, (1:45) + ii*45)));
                                                          
            Xf_mono((1:45) + ii*45)     = SPf_mono((1:45) + ii*45, (1:45) + ii*45)...
                                          *(SPfm_auv\Xfm_auv + SPfm_wamv\Xfm_wamv + SPfm_quad\Xfm_quad...
                                          - 2*(SPmono_pri((1:45) + ii*45, (1:45) + ii*45))\Xmono_pri((1:45) + ii*45));
        else
            error('Not full rank');
        end
    end
    
    % UKF_PU
    f = @(x,u) pm_mono_oosm(x,u);
    [Xf_mono, SPf_mono] = UKF_PU(Xf_mono, SPf_mono, [data.AUV.U(:,t);data.WAMV.U(:,t);data.QUAD.U(:,t)], f);

    % Full state disassembly
    for ii = 0:param.maxDelay - 1
        data.AUV.Xf([1:15, 16:21] + ii*21,t+1)                          = Xf_mono([1:15, 16:21] + ii*45);    
        data.AUV.SPf([1:15, 16:21] + ii*21, [1:15, 16:21] + ii*21,t+1)  = SPf_mono([1:15,16:21] + ii*45,[1:15,16:21] + ii*45);

        data.WAMV.Xf((1:15) + ii*15, t+1)                   = Xf_mono((16:30) + ii*45);            
        data.WAMV.SPf((1:15) + ii*15,(1:15) + ii*15, t+1)   = SPf_mono((16:30) + ii*45,(16:30) + ii*45);

        data.QUAD.Xf([1:15, 16:21] + ii*21,t+1)                         = Xf_mono([31:45, 16:21]  + ii*45);   
        data.QUAD.SPf([1:15, 16:21] + ii*21,[1:15, 16:21] + ii*21, t+1) = SPf_mono([31:45,16:21]  + ii*45, [31:45,16:21]  + ii*45);
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

PlotData(data);

clearvars -except data param map 