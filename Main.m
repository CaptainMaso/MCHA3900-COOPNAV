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
param.GPS.sigma      = eye(3)*1e0;       % Noise on gps data (m)
param.GPS.datalength = 3;

% ---- HAP parameters
param.HAP.sigma      = eye(3)*1e-4;       % Noise on HAP data (m)
param.HAP.datalength = 3;

% --- Simulation Parameters
param.tf = 2;

% --- Vehicle Uncertainties
param.AUV.SQeta = diag([1e-2 1e-2 1e-2, deg2rad([20 20 20])])./param.sensor_sample_rate;
param.AUV.SQnu  = diag([1e0 1e0 1e0,  deg2rad([40 40 40])])./param.sensor_sample_rate;

param.WAMV.SQeta = diag([1e-2 1e-2 1e-2, deg2rad([5 5 5])])./param.sensor_sample_rate;
param.WAMV.SQnu  = diag([5e-1 5e-1 5e0  deg2rad([60 60 25])])./param.sensor_sample_rate;

param.QUAD.SQeta = diag([1e-2 1e-2 1e-2, deg2rad([1 1 1])])./param.sensor_sample_rate;
param.QUAD.SQnu  = diag([2e1 2e1 2e2, deg2rad([5 5 5])])./param.sensor_sample_rate;

param.IMU.SQbias     = diag(deg2rad([0.01 0.01 0.01]));
param.IMU.SQbias0     = diag(deg2rad([1500 1500 1500]));

% --- Vehicle data parameters
param.AUV.datalength  = param.IMU.datalength + ...
                        param.HAP.datalength;

param.WAMV.datalength = param.IMU.datalength + ...
                        param.GPS.datalength  + ...
                        param.VB.datalength * map.VB.N;
                        
param.QUAD.datalength = param.IMU.datalength + ...
<<<<<<< HEAD
<<<<<<< HEAD
                        param.GPS.datalength  + ...
=======
                        param.GPS.datalength + ...
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
=======
                        param.GPS.datalength + ...
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
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

    clearvars -except data param map
    disp('Finished QUAD SIM');
end
assert(data.AUV.N == data.WAMV.N && data.AUV.N == data.QUAD.N, 'State trajectories must be same length');
data.ALL.N = data.AUV.N;
data.t = data.AUV.t;

%% Simulate Sensor Data

% Get all Data
if (param.enabled(1)); data.AUV.Y  = zeros(param.AUV.datalength,  param.tf*param.sensor_sample_rate); end
if (param.enabled(2)); data.WAMV.Y = zeros(param.WAMV.datalength, param.tf*param.sensor_sample_rate); end
if (param.enabled(3)); data.QUAD.Y = zeros(param.QUAD.datalength, param.tf*param.sensor_sample_rate); end

dt = zeros(30,1);                 
for t = 1:data.ALL.N
    dtt = tic;

    if (param.enabled(1))
        % Get AUV data
        [data.AUV.Y(:,t),SR_AUV] = getRawDataAUV(data.AUV.X(:,t), data.AUV.dnu(:,t), data.WAMV.X(1:6,t), param.IMU.gyro_bias);
        data.AUV.Y(:,t)     = data.AUV.Y(:,t) + SR_AUV*randn(size(SR_AUV,1),1);
%         data.AUV.IMU(:,t)   = data.AUV.Y(1:9,t);
%         data.AUV.HAP(:,t)   = data.AUV.Y(10:12,t);
    end
    
    if (param.enabled(2))
        % Get WAMV data
        [data.WAMV.Y(:,t),SR_WAMV] = getRawDataWAMV(data.WAMV.X(:,t), data.WAMV.dnu(:,t), param.IMU.gyro_bias);
        data.WAMV.Y(:,t)  = data.WAMV.Y(:,t) + SR_WAMV*randn(size(SR_WAMV,1),1);
%         data.WAMV.IMU(:,t)   = data.WAMV.Y(1:9,t);
%         data.WAMV.GPS(:,t)   = data.WAMV.Y(10:12,t);
%         vboffset = param.VB.datalength*map.VB.N;
%         data.WAMV.VB(:,t)    = data.WAMV.Y(13:13+vboffset-1, t);
    end
    
    if (param.enabled(3))
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
    end
    
    % TTF
    dt(1:29) = dt(2:30);
    dt(30) = toc(dtt);
    ttf = degrees2dm(sum(dt)/sum(dt ~= 0)*(data.ALL.N - t)/60);
    disp(['ETA data gen: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

%% KALMAN FILTER THE FUCK OUT OF SHIT
Mono_Sub_switch = 2;            % 0 = Sub 1 = Mono;  2 = both/compare

if Mono_Sub_switch ~= 0 && Mono_Sub_switch ~= 1 && Mono_Sub_switch ~= 2
    error('mono_sub_switch must be 0, 1, or 2, dumbass');
end

N = size(data.QUAD.Y,2);     % Length of data
n = size(data.AUV.X,1) + size(data.WAMV.X,1) + size(data.QUAD.X,1) + 9;
m = size(data.AUV.Y,1) + size(data.WAMV.Y,1) + size(data.QUAD.Y,1);
p = size(data.AUV.U,1) + size(data.WAMV.U,1) + size(data.QUAD.U,1);

data.ALL.Y = [data.AUV.Y;data.WAMV.Y;data.QUAD.Y];
data.ALL.U = [data.AUV.U;data.WAMV.U;data.QUAD.U];
data.ALL.U = [zeros(p, 1), data.ALL.U];

% Initialise space for estimated means and covariances
% data.AUV.Xf  = zeros(12, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)
% data.WAMV.Xf = zeros(12, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)
% data.QUAD.Xf = zeros(12, N+1); % Prior and Posterior states (mup(:,t) = prior, mup(:,t+1) = posterior)

data.AUV.gyrobias  = zeros(3, N+1);
data.WAMV.gyrobias = zeros(3, N+1);
data.QUAD.gyrobias = zeros(3, N+1);

data.AUV.SPf   = zeros(12,12,N+1);  % Prior and Posterior Squareroot Covariances
data.WAMV.SPf  = zeros(12,12,N+1);  % Prior and Posterior Squareroot Covariances
data.QUAD.SPf  = zeros(12,12,N+1);  % Prior and Posterior Squareroot Covariances

data.AUV.SPbias   = zeros(3,3,N+1);  % Prior and Posterior Squareroot Covariances
data.WAMV.SPbias  = zeros(3,3,N+1);  % Prior and Posterior Squareroot Covariances
data.QUAD.SPbias  = zeros(3,3,N+1);  % Prior and Posterior Squareroot Covariances

data.ALL.Xf    = zeros(n, N+1); % Mean Filtered states
data.ALL.SPf   = zeros(n,n,N+1);  % Prior and Posterior Squareroot Covariances

% Set initial values
<<<<<<< HEAD
<<<<<<< HEAD
% if (param.enabled(1)); data.AUV.SPf(:,:,1)  = 2*blkdiag(param.AUV.SQeta, param.AUV.SQnu);   data.AUV.SPbias(:,:,1)  = param.IMU.SQbias; end
% if (param.enabled(2)); data.WAMV.SPf(:,:,1) = 2*blkdiag(param.WAMV.SQeta, param.WAMV.SQnu); data.WAMV.SPbias(:,:,1) = param.IMU.SQbias; end
% if (param.enabled(3)); data.QUAD.SPf(:,:,1) = 2*blkdiag(param.QUAD.SQeta, param.QUAD.SQnu); data.QUAD.SPbias(:,:,1) = param.IMU.SQbias; end
data.ALL.SPf(:,:,1)     = 1e-3*blkdiag(param.AUV.SQeta, param.AUV.SQnu, ...
                                    param.WAMV.SQeta, param.WAMV.SQnu, ...
                                    param.QUAD.SQeta, param.QUAD.SQnu, ...
                                    param.IMU.SQbias0, param.IMU.SQbias0, param.IMU.SQbias0);
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a

    data.ALL.SPf(:,:,1)     = 2*blkdiag(param.AUV.SQeta, param.AUV.SQnu, ...
                                        param.WAMV.SQeta, param.WAMV.SQnu, ...
                                        param.QUAD.SQeta, param.QUAD.SQnu, ...
                                        param.IMU.SQbias0, param.IMU.SQbias0, param.IMU.SQbias0);
% if Mono_Sub_switch == 0     %if running sub estimators, initialise sub SPf
%     if (param.enabled(1)); data.AUV.SPf(:,:,1)  = 2*blkdiag(param.AUV.SQeta, param.AUV.SQnu);   data.AUV.SPbias(:,:,1)  = param.IMU.SQbias; end
%     if (param.enabled(2)); data.WAMV.SPf(:,:,1) = 2*blkdiag(param.WAMV.SQeta, param.WAMV.SQnu); data.WAMV.SPbias(:,:,1) = param.IMU.SQbias; end
%     if (param.enabled(3)); data.QUAD.SPf(:,:,1) = 2*blkdiag(param.QUAD.SQeta, param.QUAD.SQnu); data.QUAD.SPbias(:,:,1) = param.IMU.SQbias; end
%     Sp_prior = zeros(n,n,3);    Mu_prior = zeros(n,3);
% 
% end      
data_dist = data;   data_mono = data;
for t = 1:data.ALL.N
<<<<<<< HEAD
<<<<<<< HEAD
     dtt = tic;

    % Measurement update - Monolithic
    g  = @(x,u) mm_mono(x,u);
    [data.ALL.Xf(:,t), data.ALL.SPf(:,:,t)]     = UKF_MU(data.ALL.Y(:,t), data.ALL.Xf(:,t), data.ALL.SPf(:,:,t), data.ALL.U(:,t), g);
=======
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
    dtt = tic;
if Mono_Sub_switch ~= 0 && Mono_Sub_switch ~= 1 && Mono_Sub_switch ~= 2
    error('shits fucked');
end
<<<<<<< HEAD
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a

if Mono_Sub_switch == 0 || Mono_Sub_switch == 2    % Sub estimator stuff 
    Mono_Sub_switch_mu = 0;
    Mu_prior(:,1) = data_dist.ALL.Xf(:,t);      Mu_prior(:,2) = data_dist.ALL.Xf(:,t);      Mu_prior(:,3) = data_dist.ALL.Xf(:,t);
    Sp_prior(:,:,1) = data_dist.ALL.SPf(:,:,t);  Sp_prior(:,:,2) = data_dist.ALL.SPf(:,:,t);  Sp_prior(:,:,3) = data_dist.ALL.SPf(:,:,t);
    
    f =  @(x,u) mm_auv([data_dist.ALL.Xf(1:12,t); data_dist.ALL.Xf(13:24,t); data_dist.AUV.gyrobias(:,t)], data_dist.AUV.U(:,t));
    [Temp_prior_Mu, Temp_prior_Sp]     = UKF_MU(data_dist.AUV.Y(:,t), [data_dist.ALL.Xf(1:12,t); data_dist.ALL.Xf(37:39,t); data_dist.ALL.Xf(13:18,t)], blkdiag(data_dist.ALL.SPf(1:12,1:12,t),data_dist.ALL.SPf(37:39,37:39,t)), data_dist.AUV.U(:,t),f,Mono_Sub_switch_mu);
    Mu_prior(1:12,1) = Temp_prior_Mu(1:12);	Mu_prior(37:39,1) = Temp_prior_Mu(13:15);  Sp_prior(1:12,1:12,1) = Temp_prior_Sp(1:12,1:12);   Sp_prior(37:39,37:39,1) = Temp_prior_Sp(13:15,13:15);
    
    f =  @(x,u) mm_wamv([data_dist.ALL.Xf(13:24,t); data_dist.ALL.Xf(40:42,t)], data_dist.WAMV.U(:,t));
    [Temp_prior_Mu, Temp_prior_Sp]     = UKF_MU(data_dist.WAMV.Y(:,t), [data_dist.ALL.Xf(13:24,t); data_dist.ALL.Xf(40:42,t)], blkdiag(data_dist.ALL.SPf(13:24,13:24,t),data_dist.ALL.SPf(40:42,40:42,t)), data_dist.WAMV.U(:,t),f,Mono_Sub_switch_mu);        %QUAD MU
    Mu_prior(13:24,2) = Temp_prior_Mu(1:12); Mu_prior(40:42,2) = Temp_prior_Mu(13:15); Sp_prior(13:24,13:24,2) = Temp_prior_Sp(1:12,1:12); Sp_prior(40:42,40:42,2) = Temp_prior_Sp(13:15,13:15);
    
    f =  @(x,u) mm_quad([data_dist.ALL.Xf(25:36,t);data_dist.ALL.Xf(43:45,t); data_dist.ALL.Xf(13:18,t)],data_dist.QUAD.U(:,t));
    [Temp_prior_Mu, Temp_prior_Sp]     = UKF_MU(data_dist.QUAD.Y(:,t), [data_dist.ALL.Xf(25:36,t);data_dist.ALL.Xf(43:45,t); data_dist.ALL.Xf(13:18,t)],  blkdiag(data_dist.ALL.SPf(25:36,25:36,t),data_dist.ALL.SPf(43:45,43:45,t)), data_dist.QUAD.U(:,t),f,Mono_Sub_switch_mu);        %QUAD MU
    Mu_prior(25:36,3) = Temp_prior_Mu(1:12); Mu_prior(43:45,3) = Temp_prior_Mu(13:15); Sp_prior(25:36,25:36,3) = Temp_prior_Sp(1:12,1:12); Sp_prior(43:45,43:45,3) = Temp_prior_Sp(13:15,13:15);
    
    data_dist.ALL.SPf(:,:,t) = inv(inv(Sp_prior(:,:,1)) + inv(Sp_prior(:,:,2)) + inv(Sp_prior(:,:,3)) - 2*inv(data_dist.ALL.SPf(:,:,t)));
    data_dist.ALL.Xf(:,t) = data_dist.ALL.SPf(:,:,t)*(Sp_prior(:,:,1)\Mu_prior(:,1) + Sp_prior(:,:,2)\Mu_prior(:,2) + Sp_prior(:,:,3)\Mu_prior(:,3) - 2*inv(data_dist.ALL.SPf(:,:,t))*data_dist.ALL.Xf(:,t));
    %     % UKF_PU
    %     u = U(:,t+1);
    %     processFunc  = @(x,u) processModelMonolithic(x, u);
    %     [mu_next, S_next] = unscentedTransform(muf_mono(:,t), SPf_mono, processFunc, u);
<<<<<<< HEAD
    f = @(x,u) pm_mono(x,u);
    [data_dist.ALL.Xf(:,t+1), data_dist.ALL.SPf(:,:,t+1)] = UKF_PU(data_dist.ALL.Xf(:,t), data_dist.ALL.SPf(:,:,t), data_dist.ALL.U(:,t+1), f);

end

if Mono_Sub_switch == 1 || Mono_Sub_switch == 2 % Mono stuff   
    Mono_Sub_switch_mu = 1;
%     Measurement update - Monolithic
    g = @(x,u) mm_mono(x, u);
    [data_mono.ALL.Xf(:,t), data_mono.ALL.SPf(:,:,t)]     = UKF_MU(data_mono.ALL.Y(:,t), data_mono.ALL.Xf(:,t), data_mono.ALL.SPf(:,:,t), data_mono.ALL.U(:,t), g,Mono_Sub_switch_mu);
%     Process Update - Monolithic
    f = @(x,u) pm_mono(x,u);
    [data_mono.ALL.Xf(:,t+1), data_mono.ALL.SPf(:,:,t+1)] = UKF_PU(data_mono.ALL.Xf(:,t), data_mono.ALL.SPf(:,:,t), data_mono.ALL.U(:,t+1), f);
    f = @(x,u) pm_mono(x,u);
=======
    f = @(x,u) pm_mono(x,u);
    [data_dist.ALL.Xf(:,t+1), data_dist.ALL.SPf(:,:,t+1)] = UKF_PU(data_dist.ALL.Xf(:,t), data_dist.ALL.SPf(:,:,t), data_dist.ALL.U(:,t+1), f);

end

if Mono_Sub_switch == 1 || Mono_Sub_switch == 2 % Mono stuff   
    Mono_Sub_switch_mu = 1;
%     Measurement update - Monolithic
    g = @(x,u) mm_mono(x, u);
    [data_mono.ALL.Xf(:,t), data_mono.ALL.SPf(:,:,t)]     = UKF_MU(data_mono.ALL.Y(:,t), data_mono.ALL.Xf(:,t), data_mono.ALL.SPf(:,:,t), data_mono.ALL.U(:,t), g,Mono_Sub_switch_mu);
%     Process Update - Monolithic
    f = @(x,u) pm_mono(x,u);
    [data_mono.ALL.Xf(:,t+1), data_mono.ALL.SPf(:,:,t+1)] = UKF_PU(data_mono.ALL.Xf(:,t), data_mono.ALL.SPf(:,:,t), data_mono.ALL.U(:,t+1), f);
    f = @(x,u) pm_mono(x,u);
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
    [data_mono.ALL.Xf(:,t+1), data_mono.ALL.SPf(:,:,t+1)] = UKF_PU(data_mono.ALL.Xf(:,t), data_mono.ALL.SPf(:,:,t), data_mono.ALL.U(:,t+1), f);
    
end
    
    % TTF
    dt(1:29) = dt(2:30);
    dt(30) = toc(dtt);
    ttf = degrees2dm(sum(dt)/sum(dt ~= 0)*(data.ALL.N - t)/60);
    disp(['ETA Filtering: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

%% Resize and separate filtered data
<<<<<<< HEAD
<<<<<<< HEAD
data.ALL.Xf = data.ALL.Xf(:,1:end-1);
%%
data.AUV.Xf = data.ALL.Xf(1:12,:);
data.WAMV.Xf = data.ALL.Xf(13:24,:);
data.QUAD.Xf = data.ALL.Xf(25:36,:);

data.AUV.gyrobias = data.ALL.Xf(37:39,:);
data.WAMV.gyrobias = data.ALL.Xf(40:42,:);
data.QUAD.gyrobias = data.ALL.Xf(43:45,:);
=======
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
if Mono_Sub_switch == 0 || Mono_Sub_switch == 2% dist stuff   
    data_dist.ALL.Xf = data_dist.ALL.Xf(:,1:end-1);
    data_dist.AUV.Xf = data_dist.ALL.Xf(1:12,:);
    data_dist.WAMV.Xf = data_dist.ALL.Xf(13:24,:);
    data_dist.QUAD.Xf = data_dist.ALL.Xf(25:36,:);
end

if Mono_Sub_switch == 1 || Mono_Sub_switch == 2% Mono stuff   
    data_mono.ALL.Xf = data_mono.ALL.Xf(:,1:end-1);
    data_mono.AUV.Xf = data_mono.ALL.Xf(1:12,:);
    data_mono.WAMV.Xf = data_mono.ALL.Xf(13:24,:);
    data_mono.QUAD.Xf = data_mono.ALL.Xf(25:36,:);
end
<<<<<<< HEAD
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
=======
>>>>>>> 9cb5aa03f0120a7a4d5036432d6cb270fcdfd97a
%%
close all
PlotData(data_dist,data_mono,Mono_Sub_switch);

clearvars -except data_dist data_mono param map 