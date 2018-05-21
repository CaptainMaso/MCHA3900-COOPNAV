clear variables;

addpath('QUAD');
addpath('WAMV');
addpath('AUV');
addpath('Sensors');
addpath('Common');

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
param.VB.sigma = diag([0.1, 0.1, 0.1]);          % Noise on VB data (rad)

% --- Simulation Parameters
param.tf = 30;

%% Get Vehicle Data
% -- QUAD data
Quadrotor();
data.QUAD.raw.t = QUAD_states.Time';
data.QUAD.raw.N = length(data.QUAD.raw.t);
data.QUAD.raw.X = QUAD_states.Data(:, 1:12)';
data.QUAD.raw.Taub = QUAD_states.Data(:, 13:18)';

clearvars -except data param map

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
% Plot states (True, est. mono, est. distributed)
% First etas
f = figure(1);
% QUAD eta data
columns = 3;
rows = 6;

titles = {{'QUAD','NORTH'}, {'EAST'}, {'DOWN'}, {'ROLL '}, {'PITCH'}, {'YAW'}};
ylabels = {'N (m)', 'E (m)', 'D (m)', '$\phi$ ($^{\circ}$)', '$\theta$ ($^{\circ}$)', '$\psi$ ($^{\circ}$)'};
xlabels = {'','','','','','Time (s)'};
scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
for row=1:rows
    subplot(6,3,(row-1)*columns + 1);
    plot(data.QUAD.raw.t, data.QUAD.raw.X(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
    grid on;
    t = titles(row);
    title(t{:}, 'Interpreter', 'latex');
    ylabel(ylabels(row), 'Interpreter', 'latex');
    xlabel(xlabels(row), 'Interpreter', 'latex');
    legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
end

% -------------------- TODO  - ADD WAMV/AUV
% First nus
figure(2);

columns = 3;
rows = 6;

titles = {{'QUAD';'SURGE'}, 'SWAY', 'HEAVE', 'ROLL RATE', 'PITCH RATE', 'YAW RATE'};
ylabels = {'U (m)', 'V (m)', 'W (m)', 'P ($^{\circ}$/s)', 'Q ($^{\circ}$/s)', 'R ($^{\circ}$/s)'};
xlabels = {'','','','','','Time (s)'};
scales = [1,1,1,180/pi,180/pi,180/pi,1,1,1,180/pi,180/pi,180/pi]; 
for row=1:rows
    subplot(6,3,(row-1)*columns + 1);
    plot(data.QUAD.raw.t, data.QUAD.raw.X(row,:)*scales(row));% data.QUAD.filtered.t, data.QUAD.filtered.
    grid on;
    t = titles(row);
    title(t{:}, 'Interpreter', 'latex');
    ylabel(ylabels(row), 'Interpreter', 'latex');
    xlabel(xlabels(row), 'Interpreter', 'latex');
    legend('True', 'Filtered - Monolithic', 'Filtered - Distributed');
end

clearvars -except data param map
