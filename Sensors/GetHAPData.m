function [rAWw,SR] = GetHAPData(X_AUV, X_WAMV)
global param

rANn = X_AUV(1:3);
rWNn = X_WAMV(1:3);
rAWn = rANn - rWNn;
Rwn = eulerRotation(X_WAMV(4:6));

rAWw = Rwn*rAWn;

SR = param.HAP.sigma;