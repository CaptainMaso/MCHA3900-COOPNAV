function [Y_quad, SR_quad] = mm_wamv(X,wamv_U)
% function [Y_quad, SR_quad] = mm_wamv(wamv_etanu, wamv_U, wamv_gyrobias)
wamv_etanu = X(1:12);
wamv_gyrobias = X(13:15);
% Get dnu (body-fixed accelerations)
wamv_dnu = wamv_detanu([wamv_U; wamv_etanu]);
wamv_dnu = wamv_dnu(7:12);

[Y_quad, SR_quad]   = getRawDataWAMV(wamv_etanu, wamv_dnu, wamv_gyrobias);