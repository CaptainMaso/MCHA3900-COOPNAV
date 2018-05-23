function Yraw = GetRawData(Xm, dnuM)
global param

monolithicOffset_Xm = 12*0; % Offset 0 for AUV
monolithicOffset_dnu = 6*0;
auv_etanu = Xm(1+monolithicOffset_Xm:12+monolithicOffset_Xm);
auv_dnu = dnuM(1+monolithicOffset_dnu:6+monolithicOffset_dnu);

monolithicOffset_Xm = 12*1; % Offset 1 for WAMV
monolithicOffset_dnu = 6*1;
wamv_etanu = Xm(1+monolithicOffset_Xm:12+monolithicOffset_Xm);
wamv_dnu = dnuM(1+monolithicOffset_dnu:6+monolithicOffset_dnu);

monolithicOffset_Xm = 12*2; % Offset 2 for QUAD
monolithicOffset_dnu = 6*2;
quad_etanu = Xm(1+monolithicOffset_Xm:12+monolithicOffset_Xm);
quad_dnu = dnuM(1+monolithicOffset_dnu:6+monolithicOffset_dnu);

% ----------- AUV DATA
% Gets AUV IMU data
Y_IMU = GetIMUData([auv_etanu;auv_dnu], param.IMU.gyrobias) + blkdiag(param.IMU.acc_sigma,param.IMU.gyro_sigma,param.IMU.magn_sigma)*randn(9,1);

% Gets AUV HAP data
Y_HAP = GetHAPData([auv_etanu;auv_dnu], wamv_etanu);

% Stack and return
Y_AUV = [Y_IMU;Y_HAP];

% ------------- WAMV DATA
% Gets WAMV IMU data
Y_IMU = GetIMUData([wamv_etanu;wamv_dnu], param.IMU.gyrobias) + blkdiag(param.IMU.acc_sigma,param.IMU.gyro_sigma,param.IMU.magn_sigma)*randn(9,1);

% Gets WAMV GPS data
Y_GPS = GetGPSData([wamv_etanu;wamv_dnu]) + param.GPS.sigma*randn(3,1);

% Gets VB Data
Y_VB  = GetVBData([wamv_etanu;wamv_dnu]) + param.VB.sigma*randn(3*param.VB.N,1);

% Stack and return
Y_WAMV = [Y_IMU;Y_GPS;Y_VB];

% -------------------- QUAD DATA
% Get vector from QUAD to WAMV in b (rWQb)
Rnq     = eulerRotation(quad_etanu(4:6));
rWQn    = wamv_etanu(1:3) - quad_etanu(1:3);
rWQq    = Rnq'*rWQn;

% Gets IMU data
Y_IMU   = GetIMUData([quad_etanu;quad_dnu], param.IMU.gyrobias) + blkdiag(param.IMU.acc_sigma,param.IMU.gyro_sigma,param.IMU.magn_sigma)*randn(9,1);

% Gets GPS data
Y_GPS   = GetGPSData([quad_etanu;quad_dnu]) + param.IMU.sigma*randn;

% Gets VB Data
Y_VB    = GetVBData([quad_etanu;quad_dnu]);

vb_wamv = rWQq/norm(rWQq);

Y_VB    = [Y_VB;vb_wamv] + param.VB.sigma*randn(3*(param.VB.N+1),1);  % Adds normalised bearing vector from quad to wamv

% Gets LPS Data
Y_LPS   = GetLPSData([quad_etanu;quad_dnu]);

lps_wamv = norm(rWQn);

Y_LPS   = [Y_LPS; lps_wamv] + param.LPS.sigma*randn(param.LPS.N+1,1);     % Adds distance from quad to wamv to LPS data

% Stack and return
Y_QUAD = [Y_IMU;Y_GPS;Y_VB;Y_LPS];

% Full stack and return
Yraw = [Y_AUV; Y_WAMV; Y_QUAD];