function y_HAP = GetHAPData(X_AUV, X_WAMV)
Rwn = eulerRotation(X_WAMV(4:6));
y_HAP = Rwn*(X_AUV(1:3)-X_WAMV(1:3));