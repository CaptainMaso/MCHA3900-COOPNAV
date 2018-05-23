function y_HAP = GetHAPData(X_AUV, X_WAMV)
J = eulerRotation(XWAMV(4:6));
y_HAP = J*(X_AUV(1:3)-X_WAMV(1:3));