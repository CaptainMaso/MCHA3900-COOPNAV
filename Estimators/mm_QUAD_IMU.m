function [Y_IMU, SR_IMU] = mm_QUAD_IMU(quad_etanu,quad_U,quad_gyrobias, ~)
% Get dnu (body-fixed accelerations)
quad_dnu = quad_detanu([quad_U; quad_etanu]);
quad_dnu = quad_dnu(7:12);

[Y_IMU, SR_IMU] = GetIMUData([quad_etanu;quad_dnu], quad_gyrobias);
end