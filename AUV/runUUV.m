%% UUV model in the horizontal plane
clear
clc
global param

% Indices for degrees of freedom of interest
param.dofIdx = [1 2 3 4 6]; % N, E, D, psi(yaw)

% Mass [kg]
param.m = 10.17; 
param.g = 9.81;
% Inertia
param.IBb   = diag([0.139 0.171 0.212]);

% param.IBb   = [0.98657 0.06241 0.15094;
%                -0.06085 0.99804 -0.01492;
%                -0.15158 0.00554 0.98843];

% Centre of gravity
param.xg    = 0;                  
param.yg    = 0;
param.zg    = 0;
param.rCBb  = [param.xg;param.yg;param.zg];
param.SrCBb = skew(param.rCBb);

% Hydrodynamic parameters
param.Xudot = -16.75; param.Xu = -0.03;     param.Xauu = -0.05;
param.Yvdot = -0.032752; param.Yrdot = -16.75;  param.Yv = -0.1;    param.Yavv = -0.03;
param.Nrdot = -0.015;  param.Nr = -0.02;    param.Narr = 0;

% Generalised mass matrix (constant in body coordinates)
param.MRB = [param.m.*eye(3) -param.m*param.SrCBb;
             param.m*param.SrCBb param.IBb];
param.MRB5 = param.MRB(param.dofIdx,param.dofIdx);

MA = diag([param.Xudot param.Yvdot param.Yvdot 0.3864 0.3864]);

MRBA = MA + param.MRB5;

Dlin = diag([-param.Xauu -param.Yavv -param.Yavv -param.Narr -param.Narr]);
% Formulate linear surge-sway-yaw state-space model for control design
param.Ar = [zeros(5) eye(5);
            zeros(5) -inv(MRBA)*Dlin];
param.Br = [zeros(5); inv(MRBA)];

% outputs are position
param.Cr = [eye(5) zeros(5)];

param.Dr = 0;

MatrixAa = [param.Ar zeros(10,5);
            -param.Cr zeros(5)];
        
MatrixBa = [param.Br; zeros(5)];
MatrixCa = [param.Cr zeros(5,10)];

% Check controllability
COr = ctrb(MatrixAa,MatrixBa);

assert(rank(COr) == min(size(COr)),'System not controllable!')

%% LQR control design
Q = diag([1000 100 100000 100 1000 1 1 1 1 1    100 10 1000 100 100]);
% Q = diag([50 10 10 10 0.1 0.1 0.1 0.1     0.5 0.1 0.1 0.1]);

R = 2*eye(5);

Sysr = ss(param.Ar, param.Br, param.Cr, param.Dr);
[K,S,E] = lqi(Sysr,Q,R,zeros(15,5));

Kgain = [K(:,1:10) K(:,11:15)];

% Initial and final conditions
etai = [0; 0; 0; 0; 0; 0; 0; 0];
etad5 = [0; 0; 0; 0; 5]; % Get desired eta for the 3 dof of choice

%% check eigenvalues
eigABK = eig(MatrixAa - MatrixBa*K)
disturbances_eta = zeros(5,1);
sim('AUVsim')
% %% 
% % IMUa = IMUdata(:,1:3);
% % IMUg = IMUdata(:,4:6);
% % IMUm = IMUdata(:,7:9);
% %
% [tsize,~] = size(etanu(:,1));
% refN = etad5(:,1);
% refE = etad5(:,2);
% refD = etad5(:,3);
% refphi = etad5(:,4);
% refpsi = etad5(:,5);
% 
% close ALL HIDDEN
% 
% figure(1);
% subplot(5,1,1)
% plot(etanu(:,1),etanu(:,2),etanu(:,1),refN)
% legend('State', 'Reference');
% title('N')
% 
% subplot(5,1,2)
% plot(etanu(:,1),etanu(:,3),etanu(:,1),refE)
% legend('State', 'Reference');
% title('E')
% 
% subplot(5,1,3)
% plot(etanu(:,1),etanu(:,4),etanu(:,1),refD)
% legend('State', 'Reference');
% title('D')
% 
% subplot(5,1,4)
% plot(etanu(:,1),etanu(:,5),etanu(:,1),refphi)
% legend('State', 'Reference');
% title('phi (degrees)')
% 
% subplot(5,1,5)
% plot(etanu(:,1),etanu(:,6),etanu(:,1),refpsi)
% legend('State', 'Reference');
% title('psi (degrees)')
% 
% figure(2);
% subplot(5,1,1)
% plot(etanu(:,1),etanu(:,7))
% title('u')
% 
% subplot(5,1,2)
% plot(etanu(:,1),etanu(:,8))
% title('v')
% 
% subplot(5,1,3)
% plot(etanu(:,1),etanu(:,9))
% title('w')
% 
% subplot(5,1,4)
% plot(etanu(:,1),etanu(:,10),etanu(:,1))
% legend('true state','IMU data')
% title('p (degrees/s)')
% 
% subplot(5,1,5)
% plot(etanu(:,1),etanu(:,11),etanu(:,1))
% legend('true state','IMU data')
% title('r (degrees/s)')
% 
% % subplot(5,1,5)
% % plot(IMUg(3))
% % legend('IMU data')



