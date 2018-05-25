function [Yraw,SRraw] = GetRawData(Xm, dnuM)
global param

monolithicOffset = 12*0; % Offset 0 for AUV
auv_etanu   = Xm(1+monolithicOffset:12+monolithicOffset);
monolithicOffset = 6*0; % Offset 1 for AUV
auv_dnu     = dnuM(1+monolithicOffset:6+monolithicOffset);

monolithicOffset = 12*1; % Offset 1 for WAMV
wamv_etanu  = Xm(1+monolithicOffset:12+monolithicOffset);
monolithicOffset = 6*1; % Offset 1 for WAMV
wamv_dnu    = dnuM(1+monolithicOffset:6+monolithicOffset);

monolithicOffset = 12*2; % Offset 2 for QUAD
quad_etanu  = Xm(1+monolithicOffset:12+monolithicOffset);
monolithicOffset = 6*2;  % Offset 2 for QUAD
quad_dnu    = dnuM(1+monolithicOffset:6+monolithicOffset);

% ----------- AUV DATA
% Gets AUV IMU data
[Y_IMU,SR_IMU] = GetIMUData([auv_etanu;auv_dnu], param.IMU.gyro_bias);
Y_IMU = Y_IMU + SR_IMU*randn(size(SR_IMU,1),1);

% Gets AUV HAP data
[Y_HAP, SR_HAP] = GetHAPData([auv_etanu;auv_dnu], wamv_etanu);
Y_HAP = Y_HAP + SR_HAP*randn(size(SR_HAP,1),1);

% Stack and return
Y_AUV   = [Y_IMU;Y_HAP];
SR_AUV  = blkdiag(SR_IMU, SR_HAP);

% ------------- WAMV DATA
% Gets WAMV IMU data
[Y_IMU, SR_IMU] = GetIMUData([wamv_etanu;wamv_dnu], param.IMU.gyro_bias);
Y_IMU = Y_IMU + SR_IMU*randn(size(SR_IMU,1),1);

% Gets WAMV GPS data
[Y_GPS, SR_GPS] = GetGPSData([wamv_etanu;wamv_dnu]);
Y_GPS = Y_GPS + SR_GPS*randn(size(SR_GPS,1),1);

% Gets VB Data
[Y_VB,SR_VB]  = GetVBData([wamv_etanu;wamv_dnu]);
Y_VB = Y_VB + SR_VB*randn(size(SR_VB,1),1);

% Stack and return
Y_WAMV  = [Y_IMU;Y_GPS;Y_VB];
SR_WAMV = blkdiag(SR_IMU, SR_GPS, SR_VB);

% -------------------- QUAD DATA
% Get vector from QUAD to WAMV in b (rWQb)
Rnq     = eulerRotation(quad_etanu(4:6));
rWQn    = wamv_etanu(1:3) - quad_etanu(1:3);
rWQq    = Rnq'*rWQn;

% Gets IMU data
[Y_IMU, SR_IMU] = GetIMUData([quad_etanu;quad_dnu], param.IMU.gyro_bias);
Y_IMU = Y_IMU + SR_IMU*randn(size(SR_IMU,1),1);

% Gets GPS data
[Y_GPS, SR_GPS] = GetGPSData([quad_etanu;quad_dnu]);
Y_GPS = Y_GPS + SR_GPS*randn(size(SR_GPS,1),1);

% Gets VB Data
[Y_VB,SR_VB]    = GetVBData([quad_etanu;quad_dnu]);

if (norm(rWQq) == 0)
   vb_wamv = [0;0;0];
else
    vb_wamv = rWQq/norm(rWQq);
end

SR_VB   = blkdiag(SR_VB, param.VB.sigma);
Y_VB    = [Y_VB;vb_wamv] + SR_VB*randn(size(SR_VB,1),1);  % Adds normalised bearing vector from quad to wamv

% Gets LPS Data
[Y_LPS,SR_LPS] = GetLPSData([quad_etanu;quad_dnu]);

lps_wamv = norm(rWQn);

SR_LPS  = blkdiag(SR_LPS, param.LPS.sigma);
Y_LPS   = [Y_LPS; lps_wamv] + SR_LPS*randn(size(SR_LPS,1),1);     % Adds distance from quad to wamv to LPS data

% Stack and return
Y_QUAD  = [Y_IMU;Y_GPS;Y_VB;Y_LPS];
SR_QUAD = blkdiag(SR_IMU, SR_GPS, SR_VB, SR_LPS);

% Full stack and return
Yraw    = [Y_AUV; Y_WAMV; Y_QUAD];
SRraw   = blkdiag(SR_AUV, SR_WAMV, SR_QUAD);