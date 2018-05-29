function [y_imu,SR] = GetIMUData(in, wbias)
global param
%in(1:6) -> eta
%in(7:12) -> nu
%in(13:18) -> dnu

eta = in(1:6);
nu = in(7:12);
dnu = in(13:18);

gn = [0;0;param.g]; 

thetanb = eta(4:6);
Rnb = eulerRotation(thetanb); %-> Rnb is the rotation matrix from body to NED coords

%Accelerometer
rMBb = [0;0;0];
vBNb = nu(1:3);
wBNb = nu(4:6);

Rmb = eye(3); %-> IMU is at COG for vessel
aimu = Rmb*([eye(3), -skew(rMBb)]*dnu) + Rmb*(skew(wBNb)*(skew(wBNb)*rMBb + vBNb)-(Rnb')*gn);

%Gyro
wimu = nu(4:6) + wbias;

%Magnetometer
m = [1;0;0];                         %not sure if this is correct
mimu = (Rnb')*m;

y_imu = [aimu;wimu;mimu];
SR    = blkdiag(param.IMU.acc_sigma,param.IMU.gyro_sigma,param.IMU.magn_sigma);
