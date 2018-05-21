function [y_imu, Cjac] = IMU_MeasurementModel(X, U, ProcessModel)
% Inputs: X = States [eta;nu;biases], U = Body-fixed Actuator forces (FBb),
% ProcessModel = Function mapping X & U to Detanu
global param

T = 1/param.IMU.samplerate;

inPM = [X(1:12);U];
detanu = ProcessModel(inPM);

dnu = detanu(7:12);

y_acc = [eye(3), -skew(rMBb)]*dnu + skew(wBNb)*vBNb - Rnb'*gn;
y_gyro = blkdiag(zeros(9), eye(3))*X;
