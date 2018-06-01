function [Y_IMU, SR_IMU] = mm_AUV_IMU(auv_etanu,auv_u, auv_gyrobias, ~)
% Get dnu (body-fixed accelerations)
auv_dnu = auv_detanu([auv_u; auv_etanu]);
auv_dnu = auv_dnu(7:12);

[Y_IMU, SR_IMU] = GetIMUData([auv_etanu;auv_dnu], auv_gyrobias);
end

