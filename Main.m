clear variables;

addpath('QUAD');
addpath('WAMV');
addpath('AUV');
addpath('Sensors');

global param map data
%% Parameter Initialisation
% -- Universal Parameters
% --- World Parameters
param.g = 9.81;

% --- Sensor Parameters
param.sensor_sample_rate = 10;  % Sample rate (Hz)

% ---- LPS parameters
param.LPS.sigma = 0.1;          % Noise on LPS data (m)

% ---- Visual Bearing (VB) parameters
param.VB.sigma = diag([0.1, 0.1, 0.1]);          % Noise on VB data (m)

% --- Simulation Parameters
param.tf = 30;

%% Get Vehicle Data
% -- QUAD data
Quadrotor();
data.QUAD.raw.t = QUAD_states.Time';
data.QUAD.raw.N = length(data.QUAD.raw.t);
data.QUAD.raw.X = QUAD_states.Data(:, 1:12)';
data.QUAD.raw.Taub = QUAD_states.Data(:, 13:18)';
clear QUAD_states tout

% -- WAMV data
% TODO: Implement this
 
% -- AUV data
% TODO: Implement this

%% Simulate Sensor Data
% Get Map data
Map();

% -- QUAD measurement simulation
% --- LPS data
data.QUAD.LPS = zeros(map.LPS.N, data.QUAD.raw.N);
for ii = 1:data.QUAD.raw.N
    data.QUAD.LPS(:,ii) = GetLPSData([data.QUAD.raw.X(:,ii); data.QUAD.raw.Taub(:,ii)]);
end

% --- VB data
data.QUAD.VB = zeros(3, map.VB.N, data.QUAD.raw.N);
for ii = 1:data.QUAD.raw.N
    data.QUAD.VB(:,:,ii) = GetVBData([data.QUAD.raw.X(:,ii); data.QUAD.raw.Taub(:,ii)]);
end

% -- WAMV measurement simulation

% -- AUV measurement simulation

% -- Relative measurement simulation

%% KALMAN FILTER THE FUCK OUT OF SHIT
% Lol

%% PLOT ALL THE THINGS (For now)
