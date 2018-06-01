function [Y_IMU, SR_IMU] = mm_WAMV_IMU(wamv_etanu, wamv_u, wamv_gyrobias)
% Get dnu (body-fixed accelerations)
wamv_dnu = wamv_detanu([wamv_u; wamv_etanu]);
wamv_dnu = wamv_dnu(7:12);

[Y_IMU, SR_IMU] = GetIMUData([wamv_etanu;wamv_dnu], wamv_gyrobias);
end