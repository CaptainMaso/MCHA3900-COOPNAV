%%Finding Jacobians Symbolically for IMU and GPS
syms N E D phi theta psi u v w p q r du dv dw dp dq dr g abias anoise wbias wnoise mbias mnoise GPSnoise
assume([N E D phi theta psi u v w p q r du dv dw dp dq dr g abias anoise wbias wnoise mbias mnoise GPSnoise], 'real');
eta = [N;E;D;phi;theta;psi];
nu = [u;v;w;p;q;r];
dnu = [du;dv;dw;dp;dq;dr];
gn = [0;0;g]; 
thetanb = eta(4:6);
Rnb = eulerRotation(thetanb); %-> Rnb is the rotation matrix from body to NED coords
%Accelerometer
rMBb = [0;0;0];
vBNb = nu(1:3);
wBNb = nu(4:6);
Rmb = eye(3); 
a = [eye(3), -skew(rMBb)];
aimu = Rmb*(a*dnu) + Rmb*(skew(wBNb)*(skew(wBNb)*rMBb + vBNb)-Rnb'*gn) + abias + anoise;
%Gyro
wimu = nu(4:6) + wbias + wnoise;
%Magnetometer
m = [1;0;0];                         
mimu = (Rnb')*m + mbias + mnoise;


y_imu = [aimu;wimu;mimu];

acc_jac  = jacobian(aimu,[eta;nu;dnu])
gyro_jac = jacobian(wimu,[eta;nu;dnu])
mag_jac = jacobian(mimu,[eta;nu;dnu])

y_gps = eta(1:3) + GPSnoise;
gps_jaceta = jacobian(y_gps,[eta;nu;dnu])
