function [Y_LPS, SR_LPS] = mm_QUAD_LPS(quad_etanu, ~, ~, wamv_eta)
global param
rWQn    = wamv_eta(1:3) - quad_etanu(1:3);

% Gets LPS Data
[Y_LPS,SR_LPS] = GetLPSData(quad_etanu);

lps_wamv = norm(rWQn); 
SR_LPS  = blkdiag(SR_LPS, param.LPS.sigma);
Y_LPS   = [Y_LPS; lps_wamv];        % Adds distance from quad to wamv to LPS data