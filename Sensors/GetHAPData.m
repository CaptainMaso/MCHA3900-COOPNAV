function [y_HAP,SR] = GetHAPData(X_AUV, X_WAMV)
global param

Rwn = eulerRotation(X_WAMV(4:6));
y_HAP = Rwn*(X_AUV(1:3)-X_WAMV(1:3));

SR = param.HAP.sigma;