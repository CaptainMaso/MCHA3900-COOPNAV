function [Y_QUAD, SR_QUAD] = measurementModelQUAD(Xm, U_QUAD)

monolithicOffset = 12*2; % Offset 2 for QUAD
quad_etanu = Xm(1+monolithicOffset:12+monolithicOffset);
quad_dnu = WAMV_detanu([U_QUAD;quad_etanu]);

monolithicOffset = 12*1; % Offset 1 for WAMV
wamv_etanu = Xm(1+monolithicOffset:12+monolithicOffset);

monolithicOffset = 12*3 + 3*2; % Offset 3 for biases, 2 for QUAD
quad_gyrobias = Xm(1+monolithicOffset:3+monolithicOffset)

% Get vector from QUAD to WAMV in b (rWQb)
Rnq     = eulerRotation(quad_etanu(4:6));
rWQn    = wamv_etanu(1:3) - quad_etanu(1:3);
rWQq    = Rnq'*rWQn;

% Gets IMU data
[Y_IMU, SR_IMU] = GetIMUData([quad_etanu;quad_dnu], quad_gyrobias);

% Gets GPS data
[Y_GPS, SR_GPS] = GetGPSData([quad_etanu;quad_dnu]);

% Gets VB Data
[Y_VB,SR_VB]    = GetVBData([quad_etanu;quad_dnu]);

if (norm(rWQq) == 0)
   vb_wamv = [0;0;0];
else
    vb_wamv = rWQq/norm(rWQq);
end

SR_VB   = blkdiag(SR_VB, param.VB.sigma);
Y_VB    = [Y_VB;vb_wamv];           % Adds normalised bearing vector from quad to wamv

% Gets LPS Data
[Y_LPS,SR_LPS] = GetLPSData([quad_etanu;quad_dnu]);

lps_wamv = norm(rWQn);

SR_LPS  = blkdiag(SR_LPS, param.LPS.sigma);
Y_LPS   = [Y_LPS; lps_wamv];        % Adds distance from quad to wamv to LPS data

% Stack and return
Y_QUAD  = [Y_IMU;Y_GPS;Y_VB;Y_LPS];
SR_QUAD = blkdiag(SR_IMU, SR_GPS, SR_VB, SR_LPS);