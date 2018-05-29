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
param.enabled = [1,1,1]; % AUV, WAMV, QUAD

% -- Universal Parameters
% --- World Parameters
param.g = 9.81;

% --- Sensor Parameters
param.sensor_sample_rate = 200;  % Sample rate (Hz)

% ---- LPS parameters
param.LPS.sigma = 1e-2;          % Noise on LPS data (m)
param.LPS.datalength = 1;

% ---- Visual Bearing (VB) parameters
param.VB.sigma      = eye(3)*1e-3;      % Noise on VB data (rad)
param.VB.datalength = 3;

% ---- IMU parameters
param.IMU.acc_sigma = eye(3)*1e-4;      % Noise on accelerometer data (m/s^2)
param.IMU.gyro_sigma = eye(3)*1e-6;     % Noise on gyro data (rad/s)
param.IMU.gyro_bias  = [0.4, -0.2, 0.6]';
param.IMU.magn_sigma = eye(3)*1e-5;
param.IMU.datalength = 9;

% ---- GPS parameters
param.GPS.sigma      = eye(3)*1e-3;       % Noise on gps data (m)
param.GPS.datalength = 3;

% ---- HAP parameters
param.HAP.sigma      = eye(3)*1e-3;       % Noise on HAP data (m)
param.HAP.datalength = 3;

% --- Simulation Parameters
param.tf = 10;

% --- Vehicle Uncertainties
param.AUV.SQeta = diag([1e-1 1e-1 1e-1, deg2rad([50 50 50])])./param.sensor_sample_rate*1e2;
param.AUV.SQnu  = diag([1e-0 1e-0 1e0,  deg2rad([50 50 50])])./param.sensor_sample_rate*1e2;

param.WAMV.SQeta = diag([1e1 1e1 1e1, deg2rad([200 200 40])])./param.sensor_sample_rate*2e2;
param.WAMV.SQnu  = diag([1e1 1e1 1e1,  deg2rad([200 200 40])])./param.sensor_sample_rate*2e2;

param.QUAD.SQeta = diag([1e-1 1e-1 1e-1, deg2rad([1 1 1])])./param.sensor_sample_rate;
param.QUAD.SQnu  = diag([1e-0 1e-0 1e-0, deg2rad([5 5 5])])./param.sensor_sample_rate;

param.IMU.SQbias     = diag(deg2rad([0.1 0.1 0.1]))./param.sensor_sample_rate;
param.IMU.SQbias0     = diag(deg2rad([20 20 20]));

% --- Vehicle data parameters
param.AUV.datalength  = param.IMU.datalength + ...
                        param.HAP.datalength;

param.WAMV.datalength = param.IMU.datalength + ...
                        param.GPS.datalength  + ...
                        param.VB.datalength * map.VB.N;
                        
param.QUAD.datalength = param.IMU.datalength + ...
                        param.GPS.datalength;%  + ...
                        %param.VB.datalength*(map.VB.N+1) + ...
                        %param.LPS.datalength*(map.LPS.N+1);

% Get Map data
Map();
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
    ttf = degrees2dm(sum(dt)/30*(data.ALL.N - t)/60);
    disp(['ETA data gen: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

%% KALMAN FILTER THE FUCK OUT OF SHIT
N = size(data.ALL.Y,2);     % Length of data
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
% 
% data.AUV.gyrobias  = zeros(3, N+1);
% data.WAMV.gyrobias = zeros(3, N+1);
% data.QUAD.gyrobias = zeros(3, N+1);
% 
% data.AUV.SPf   = zeros(12,12,N+1);  % Prior and Posterior Squareroot Covariances
% data.WAMV.SPf  = zeros(12,12,N+1);  % Prior and Posterior Squareroot Covariances
% data.QUAD.SPf  = zeros(12,12,N+1);  % Prior and Posterior Squareroot Covariances
% 
% data.AUV.SPbias   = zeros(3,3,N+1);  % Prior and Posterior Squareroot Covariances
% data.WAMV.SPbias  = zeros(3,3,N+1);  % Prior and Posterior Squareroot Covariances
% data.QUAD.SPbias  = zeros(3,3,N+1);  % Prior and Posterior Squareroot Covariances

data.ALL.Xf    = zeros(n, N+1); % Mean Filtered states
data.ALL.SPf   = zeros(n,n,N+1);  % Prior and Posterior Squareroot Covariances

% Set initial values
% if (param.enabled(1)); data.AUV.SPf(:,:,1)  = 2*blkdiag(param.AUV.SQeta, param.AUV.SQnu);   data.AUV.SPbias(:,:,1)  = param.IMU.SQbias; end
% if (param.enabled(2)); data.WAMV.SPf(:,:,1) = 2*blkdiag(param.WAMV.SQeta, param.WAMV.SQnu); data.WAMV.SPbias(:,:,1) = param.IMU.SQbias; end
% if (param.enabled(3)); data.QUAD.SPf(:,:,1) = 2*blkdiag(param.QUAD.SQeta, param.QUAD.SQnu); data.QUAD.SPbias(:,:,1) = param.IMU.SQbias; end
data.ALL.SPf(:,:,1)     = 2*blkdiag(param.AUV.SQeta, param.AUV.SQnu, ...
                                    param.WAMV.SQeta, param.WAMV.SQnu, ...
                                    param.QUAD.SQeta, param.QUAD.SQnu, ...
                                    param.IMU.SQbias0, param.IMU.SQbias0, param.IMU.SQbias0);

for t = 1:data.ALL.N
%     dtt = tic;
%     
%    % Measurement update
%    g = @(x,u) measurementModelMonolithic(x,u);
%    [muf(:,t), SPf(:,:,t)]     = UKF_MU(data.ALL.Y(:,t), mup(:,t), SPp(:,:,t), U(:,t), g);
%    
%    % Process Update
%    f = @(x,u) processModelMonolithic(x,u);
%    [mup(:,t+1), SPp(:,:,t+1)] = UKF_PU(muf(:,t), SPf(:,:,t), U(:,t+1), f);
%    

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
    
   % TTF
   dt(1:29) = dt(2:30);
   dt(30) = toc(dtt);
   ttf = degrees2dm(sum(dt)/30*(data.ALL.N - t)/60);
   disp(['ETA Filtering: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
       
end
    % Measurement update - Monolithic
    g = @(x,u) mm_mono(x, u);
    [data.ALL.Xf(:,t), data.ALL.SPf(:,:,t)]     = UKF_MU(data.ALL.Y(:,t), data.ALL.Xf(:,t), data.ALL.SPf(:,:,t), data.ALL.U(:,t), g);

    % Process Update - Monolithic
    f = @(x,u) pm_mono(x,u);
    [data.ALL.Xf(:,t+1), data.ALL.SPf(:,:,t+1)] = UKF_PU(data.ALL.Xf(:,t), data.ALL.SPf(:,:,t), data.ALL.U(:,t+1), f);
    
    % TTF
    dt(1:29) = dt(2:30);
    dt(30) = toc(dtt);
    ttf = degrees2dm(sum(dt)/30*(data.ALL.N - t)/60);
    disp(['ETA Filtering: ' num2str(ttf(1), '%.0f') 'm ' num2str(ttf(2), '%.2f') 's'])
end

%% Resize and separate filtered data
data.ALL.Xf = data.ALL.Xf(:,1:end-1);

data.AUV.Xf = data.ALL.Xf(1:12,:);
data.WAMV.Xf = data.ALL.Xf(13:24,:);
data.QUAD.Xf = data.ALL.Xf(25:36,:);
%%
PlotData(data);

clearvars -except data param map