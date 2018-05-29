function [Y_AUV,SR_AUV] = getRawDataAUV(auv_etanu, auv_dnu, wamv_eta, gyrobias)
% Gets AUV IMU data
[Y_IMU, SR_IMU] = GetIMUData([auv_etanu;auv_dnu], gyrobias);

% Gets AUV HAP data
[Y_HAP, SR_HAP] = GetHAPData([auv_etanu;auv_dnu], wamv_eta);

% Stack and return
Y_AUV   = [Y_IMU;Y_HAP];
SR_AUV  = blkdiag(SR_IMU, SR_HAP);