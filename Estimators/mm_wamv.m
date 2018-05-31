function [Y_quad, SR_quad] = mm_wamv(wamv_etanu, wamv_U, wamv_gyrobias)
% Get dnu (body-fixed accelerations)
wamv_dnu = wamv_detanu([wamv_U; wamv_etanu]);
wamv_dnu = wamv_dnu(7:12);

[Y_quad, SR_quad]   = getRawDataWAMV(wamv_etanu, wamv_dnu, wamv_gyrobias);