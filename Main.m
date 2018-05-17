clear all;

addpath('QUAD');
addpath('WAMV');
addpath('AUV');
addpath('Sensors');

global param
%% Parameter Initialisation
% -- Universal Parameters
% --- World Parameters
param.g = 9.81;

% --- Sensor Parameters
param.sensor_sample_rate = 1000;

% --- Simulation Parameters
param.tf = 30;

%% Get Vehicle Data
% -- QUAD data
Quadrotor();

% -- WAMV data
% TODO: Implement this
 
% -- AUV data
% TODO: Implement this

%% Simulate Sensor Data
% -- QUAD measurement simulation

% -- WAMV measurement simulation

% -- AUV measurement simulation

% -- Relative measurement simulation

%% KALMAN FILTER THE FUCK OUT OF SHIT
% Lol

%% PLOT ALL THE THINGS (For now)
