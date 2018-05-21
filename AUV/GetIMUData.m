function y_imu = GetIMUData(in)
global param
%in(1:6) -> eta
%in(7:12) -> nu
%in(13:18) -> dnu

eta = in(1:6);
nu = in(7:12);
dnu = in(13:18);

gn = [0;0;param.g]; 
accsigma = 1;
gyrosigma = 0.01;
magsigma = 0.1;

thetanb = eta(4:6);
Rnb = eulerRotation(thetanb); %-> Rnb is the rotation matrix from body to NED coords

%Accelerometer
a = [dnu(1);dnu(2);dnu(3)];      %NED accelerations
Rmb = eye(3); %-> IMU is at COG for vessel
abias = [0.1;0.1;0.1];
anoise = randn(3,1).*accsigma;
aimu = Rmb*a + Rmb*(skew(nu(4:6))*nu(1:3)-Rnb'*gn) + abias + anoise;

%Gyro
wbias = [1;1;1];
wnoise = randn(3,1).*gyrosigma;
wimu = nu(4:6) + wbias + wnoise;

%Magnetometer
m = Rnb(1:3)';                         %not sure if this is correct
mbias = [0.1;0.1;0.1];                       %local magnetic disturbance
mnoise = randn(3,1).*magsigma;
mimu = Rnb*m + mbias + mnoise;

y_imu = [aimu;wimu;mimu];

