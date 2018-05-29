function [Y_auv, SR_auv] = mm_auv(auv_etanu, auv_U, wamv_eta, auv_gyrobias)
% Get dnu (body-fixed accelerations)
auv_dnu = auv_detanu([auv_U; auv_etanu]);
auv_dnu = auv_dnu(7:12);

[Y_auv, SR_auv]   = getRawDataAUV(auv_etanu, auv_dnu, wamv_eta, auv_gyrobias);