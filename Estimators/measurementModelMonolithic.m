function [Ym,SR] = measurementModelMonolithic(Xm, Um)
                    % Assume full 6DOF, unused states should have restoring forces on them
U_AUV   = Um(1:6);  % Assume fully actuated, underactuated systems should pad desired forces
                    % with zeros
U_WAMV  = Um(7:12);

U_QUAD  = Um(13:18);

%% AUV Data
monolithicOffset = 12*0; % Offset 0 for AUV
auv_etanu = Xm(1+monolithicOffset:12+monolithicOffset);
auv_dnu = auv_detanu([U_AUV;auv_etanu]);

monolithicOffset = 12*1; % Offset 1 for WAMV
wamv_etanu = Xm(1+monolithicOffset:12+monolithicOffset);

monolithicOffset = 12*3 + 3*0; % Offset 3 for biases, 0 for AUV
auv_gyrobias = Xm(1+monolithicOffset:3+monolithicOffset);

% Gets AUV IMU data
[Y_IMU, SR_IMU] = GetIMUData([auv_etanu;auv_dnu], 0*auv_gyrobias);

% Gets AUV HAP data
%[Y_HAP, SR_HAP] = GetHAPData([auv_etanu;auv_dnu], wamv_etanu);

% Stack and return
% Y_AUV   = [Y_IMU;Y_HAP];
% SR_AUV  = blkdiag(SR_IMU, SR_HAP);
Y_AUV   = [Y_IMU];
SR_AUV  = blkdiag(SR_IMU);

%% WAMV Data
monolithicOffset = 12*1; % Offset 1 for WAMV
wamv_etanu = Xm(1+monolithicOffset:12+monolithicOffset);
wamv_dnu = wamv_detanu([U_WAMV;wamv_etanu]);

monolithicOffset = 12*3 + 3*1; % Offset 3 for biases, 1 for WAMV
wamv_gyrobias = Xm(1+monolithicOffset:3+monolithicOffset);

% ------------- WAMV DATA
% Gets WAMV IMU data
[Y_IMU, SR_IMU] = GetIMUData([wamv_etanu;wamv_dnu], 0*wamv_gyrobias);

% Gets WAMV GPS data
%[Y_GPS, SR_GPS] = GetGPSData([wamv_etanu;wamv_dnu]);

% Gets VB Data
%[Y_VB,SR_VB]  = GetVBData([wamv_etanu;wamv_dnu]);

% Stack and return
% Y_WAMV  = [Y_IMU;Y_GPS;Y_VB];
% SR_WAMV = blkdiag(SR_IMU, SR_GPS, SR_VB);
Y_WAMV  = [Y_IMU];
SR_WAMV = blkdiag(SR_IMU);

%% QUAD Data
global param
monolithicOffset = 12*2; % Offset 2 for QUAD
quad_etanu = Xm(1+monolithicOffset:12+monolithicOffset);
quad_dnu = quad_detanu([U_QUAD;quad_etanu]);

monolithicOffset = 12*1; % Offset 1 for WAMV
wamv_etanu = Xm(1+monolithicOffset:12+monolithicOffset);

monolithicOffset = 12*3 + 3*2; % Offset 3 for biases, 2 for QUAD
quad_gyrobias = Xm(1+monolithicOffset:3+monolithicOffset);

% Get vector from QUAD to WAMV in b (rWQb)
% Rnq     = eulerRotation(quad_etanu(4:6));
% rWQn    = wamv_etanu(1:3) - quad_etanu(1:3);
% rWQq    = Rnq'*rWQn;

% Gets IMU data
[Y_IMU, SR_IMU] = GetIMUData([quad_etanu;quad_dnu], 0*quad_gyrobias);

% Gets GPS data
% [Y_GPS, SR_GPS] = GetGPSData([quad_etanu;quad_dnu]);

% Gets VB Data
% [Y_VB,SR_VB]    = GetVBData([quad_etanu;quad_dnu]);
% 
% if (norm(rWQq) == 0)
%    vb_wamv = [0;0;0];
% else
%     vb_wamv = rWQq/norm(rWQq);
% end

% SR_VB   = blkdiag(SR_VB, param.VB.sigma);
% Y_VB    = [Y_VB;vb_wamv];           % Adds normalised bearing vector from quad to wamv
% 
% % Gets LPS Data
% [Y_LPS,SR_LPS] = GetLPSData([quad_etanu;quad_dnu]);
% 
% lps_wamv = norm(rWQn);

%SR_LPS  = blkdiag(SR_LPS, param.LPS.sigma);
%Y_LPS   = [Y_LPS; lps_wamv];        % Adds distance from quad to wamv to LPS data

% Stack and return
% Y_QUAD  = [Y_IMU;Y_GPS;Y_VB;Y_LPS];
% SR_QUAD = blkdiag(SR_IMU, SR_GPS, SR_VB, SR_LPS);
Y_QUAD  = [Y_IMU];
SR_QUAD = blkdiag(SR_IMU);

%% Stack and Return
Ym = [Y_AUV;Y_WAMV;Y_QUAD];
SR = blkdiag(SR_AUV, SR_WAMV, SR_QUAD);