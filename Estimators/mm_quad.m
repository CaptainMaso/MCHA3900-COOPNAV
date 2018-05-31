function [Y_quad, SR_quad] = mm_quad(quad_etanu, quad_U, quad_gyrobias, wamv_eta)
% Get dnu (body-fixed accelerations)
quad_dnu = quad_detanu([quad_U; quad_etanu]);
quad_dnu = quad_dnu(7:12);

[Y_quad, SR_quad]   = getRawDataQUAD(quad_etanu, quad_dnu, wamv_eta, quad_gyrobias);