function [rAWw,SR] = mm_AUV_HAP(auv_etanu, ~, ~, wamv_eta)
global param
rANn = auv_etanu(1:3);
rWNn = wamv_eta(1:3);
rAWn = rANn - rWNn;
Rwn = eulerRotation(wamv_eta(4:6));

rAWw = Rwn*rAWn;

SR = param.HAP.sigma;