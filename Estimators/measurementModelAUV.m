function [Y_AUV,SR_AUV] = measurementModelAUV(Xm, U_AUV)

monolithicOffset = 12*0; % Offset 0 for AUV
auv_etanu = Xm(1+monolithicOffset:12+monolithicOffset);
auv_dnu = auv_detanu([U_AUV;auv_etanu]);

monolithicOffset = 12*1; % Offset 1 for WAMV
wamv_etanu = Xm(1+monolithicOffset:12+monolithicOffset);

monolithicOffset = 12*3 + 3*0; % Offset 3 for biases, 0 for AUV
auv_gyrobias = Xm(1+monolithicOffset:3+monolithicOffset);

% Gets AUV IMU data
[Y_IMU, SR_IMU] = GetIMUData([auv_etanu;auv_dnu], auv_gyrobias);

% Gets AUV HAP data
[Y_HAP, SR_HAP] = GetHAPData([auv_etanu;auv_dnu], wamv_etanu);

% Stack and return
Y_AUV   = [Y_IMU;Y_HAP];
SR_AUV  = blkdiag(SR_IMU, SR_HAP);