%%Finding Jacobians Symbolically for IMU and GPS
syms N E D phi theta psi u v w p q r du dv dw dp dq dr g abias anoise wbias wnoise mbias mnoise GPSnoise real 
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
aimu = Rmb*(a*dnu) + Rmb*(skew(wBNb)*(skew(wBNb)*rMBb + vBNb)-Rnb'*gn);
%Gyro
wimu = nu(4:6) + wbias + wnoise;
%Magnetometer
m = [1;0;0];                         
mimu = (Rnb')*m + mbias + mnoise;
y_imu = [aimu;wimu;mimu];

imu_jaceta = jacobian(y_imu,eta)
imu_jacnu = jacobian(y_imu,nu)
imu_jacdnu = jacobian(y_imu,dnu)

y_gps = eta(1:3) + GPSnoise;
gps_jaceta = jacobian(y_gps,eta)
gps_jacnu = jacobian(y_gps,nu)
gps_jacdnu = jacobian(y_gps,dnu)
