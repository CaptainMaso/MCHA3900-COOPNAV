function [XmfHist, SmfHist] = MonolithicUKF(Xm0, Sm0, Ym, Um, processModelM, measurementModelM)
% 
% Input Arguments:
%   Xm0:                Initial monolithic state vector
%   Sm0:                Initial monolithic square-root covariance matrix
%   Ym:                 Data input in monolithic format
%   processModelM:      Function that takes a monolithic state vector and 
%                           monolithic input vector, and returns the t+1 
%                           state vector
%   measurementModelM:  Function that takes in a monolithic state vector
%                           and a monolithic input vector and returns the
%                           estimated measurements at time 't'
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
%   XmfHist = Monolithic filtered state vector at every timestep
%   SmfHist = Monolithic filtered square-root covariance matrix at every
%               timestep

N = size(Ym,2);     % Length of data
n = size(Xm0,1);    % Size of state vector
m = size(Um,1);     % Size of input vector (Required to be n-n_biases)
p = size(Ym,1);     % Size of output vector

% Initialise space for estimated means and covariances
mup = zeros(n, N+1);
muf = zeros(n, N);

Sp  = zeros(n,n,N+1);
Sf  = zeros(n,n,N+1);

% Set initial values
mup(:,1)    = Xm0;
Sp(:,:,1)   = Sm0;

% Run UKF algorithm
for t = 1:N
   % Measurement Update
   jointFunc = @(x) augmentIdentityAdapter(measurementModelM, x, Um(:,t));
   [muxy,Syx] = unscentedTransform(mup(:,t), Sp(:,:,t), jointFunc);
   [muf(:,t), Sf(:,:,t)] = conditionGaussionOnMarginal(muxy, Syx, Ym(:,t));
   
   % Prediction Update
   processFunc = @(x) processModelM(muf(:,t), Um(:,t));
   [mup(:,t+1), Sp(:,:,t+1)] = unscentedTransform(muf(:,t), Sf(:,:,t), processFunc);
end

XmfHist = muf;
SmfHist = Sf;


