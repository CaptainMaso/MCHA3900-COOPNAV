function [XfHist, SfHist] = UKF(X0, S0, Y, U, processModel, measurementModel)
% 
% Input Arguments:
%   X0:                Initial state vector
%   S0:                Initial square-root covariance matrix
%   Y:                 Data input in format
%   processModel:      Function that takes a state vector and input vector,
%                           and returns the t+1 state vector
%   measurementModel:  Function that takes in a state vector and a 
%                           input vector and returns the estimated 
%                           measurements at time 't'
%
% Monolithic Format is of the form:
%   M = [AUV; WAMV; QUAD; (Parameters to estimate)]
% 
%   whose measurments are in turn broken up into:
%   Y_AUV   = [Y_imu; Y_hap];
%   Y_WAMV  = [Y_imu; Y_gps; Y_vb;]
%   Y_QUAD  = [Y_imu; Y_gps; Y_vb; Y_lps];
%
% Output:
%   XfHist = Filtered state vector at every timestep
%   SfHist = Filtered square-root covariance matrix at every timestep

N = size(Y,2);     % Length of data
n = size(X0,1);    % Size of state vector
m = size(U,1);     % Size of input vector (Required to be n-n_biases)
p = size(Y,1);     % Size of output vector

% Initialise space for estimated means and covariances
mup = zeros(n, N+1);
muf = zeros(n, N);

Sp  = zeros(n,n,N+1);
Sf  = zeros(n,n,N+1);

% Set initial values
mup(:,1)    = X0;
Sp(:,:,1)   = S0;

% Run UKF algorithm
for t = 1:N
    [muf(:,t), Sf(:,:,t)]   = UKF_MU(Y(:,t), mup(:,t), Sp(:,:,t), U(:,t-1), measurementModel);
    [mup(:,t+1), Sp(:,:,t)] = UKF_PU(muf(:,t), Sf(:,:,t), U(:,t), processModel);
end

XfHist = muf;
SfHist = Sf;


