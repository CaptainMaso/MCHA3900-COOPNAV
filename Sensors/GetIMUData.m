function y_imu = GetIMUData(in)
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
abias = [1;1;1];
anoise = param.IMU.accsigma*randn(3,1);
aimu = Rmb*([eye(3), -skew(rMBb)]*dnu) + Rm*(skew(wBNb)*(skew(wBNb)*rMBb + vBNb)-Rnb'*gn) + abias + anoise;

%Gyro
wbias = [1;1;1];
wnoise = param.IMU.gyrosigma*randn(3,1);
wimu = nu(4:6) + wbias + wnoise;

%Magnetometer
m = [1;0;0];                         %not sure if this is correct
mbias = [0;0;0];                       %local magnetic disturbance
mnoise = param.IMU.magnetometer*randn(3,1);
mimu = (Rnb')*m + mbias + mnoise;

y_imu = [aimu;wimu;mimu];

