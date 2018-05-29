function [Y_WAMV,SR_WAMV] = getRawDataWAMV(wamv_etanu, wamv_dnu, gyrobias)
% ------------- WAMV DATA
% Gets WAMV IMU data
[Y_IMU, SR_IMU] = GetIMUData([wamv_etanu;wamv_dnu], gyrobias);

% Gets WAMV GPS data
[Y_GPS, SR_GPS] = GetGPSData([wamv_etanu;wamv_dnu]);

% Gets VB Data
[Y_VB,SR_VB]    = GetVBData([wamv_etanu;wamv_dnu]);

% Stack and return
Y_WAMV  = [Y_IMU; Y_GPS; Y_VB];
SR_WAMV = blkdiag(SR_IMU, SR_GPS, SR_VB);