addpath('D:\Uni\2018\MCHA3900\Project\Github\Common')
%% UUV model in the horizontal plane
global param

param.tf = 60;
param.sensor_sample_rate = 100;
param.g = 9.81;
param.WAMV.T = 100;

% Indices for degrees of freedom of interest
param.WAMV.dofIdx = [1 2 6]; % N, E, psi(yaw)

% Mass [kg]
param.WAMV.m = 200; 

param.WAMV.b = 2.44; % beam
param.WAMV.l = 3.85; % length
param.WAMV.h = 1.28; % height

% Inertia
param.WAMV.Rr     = 0.5;                  % Yaw gyradius [m]
param.WAMV.Ix = param.WAMV.m*param.WAMV.b*0.35;
param.WAMV.Iy = param.WAMV.m*param.WAMV.l*0.35;
param.WAMV.Iz    = param.WAMV.m*param.WAMV.Rr^2; 	% Yaw moment of inertia [kg m^2]
param.WAMV.IBb   = diag([param.WAMV.Ix,param.WAMV.Iy,param.WAMV.Iz]);

% Centre of gravity
param.WAMV.xg    = 0.2;                  
param.WAMV.yg    = 0;
param.WAMV.zg    = 0.2;
param.WAMV.rCBb  = [param.WAMV.xg;param.WAMV.yg;param.WAMV.zg];
param.WAMV.SrCBb = skew(param.WAMV.rCBb);

%Hydrodynamic Constants

param.WAMV.Xu = -95; param.WAMV.Xdu = -150; param.WAMV.Xuu = -268;
param.WAMV.Yv = -61; param.WAMV.Ydr = -36; param.WAMV.Ydv = -264; param.WAMV.Yvv = -164; 
param.WAMV.Nr = -105;  param.WAMV.Ndr = -97; param.WAMV.Nrr = 0;

%param.WAMV.Dlin = -diag([param.WAMV.Xu,param.WAMV.Yv,param.WAMV.Zw,param.WAMV.Kp,param.WAMV.Mq,param.WAMV.Nr]);
%param.WAMV.MA = -diag([param.WAMV.Xdu,param.WAMV.Ydv,param.WAMV.Zdw,param.WAMV.Kdp,param.WAMV.Mdq,param.WAMV.Ndr]); 

% Generalised mass matrix (constant in body coordinates)
param.WAMV.MRB = [param.WAMV.m*eye(3), -param.WAMV.m*param.WAMV.SrCBb;
            param.WAMV.m*param.WAMV.SrCBb, param.WAMV.IBb];        %from 3.79;
param.WAMV.MRB3 = param.WAMV.MRB(param.WAMV.dofIdx,param.WAMV.dofIdx);

% Formulate linear surge-sway-yaw state-space model for control design
param.WAMV.Ar = [zeros(3), eye(3); zeros(3), -inv(param.WAMV.MRB3)];
param.WAMV.Br = [zeros(3); inv(param.WAMV.MRB3)];
param.WAMV.Cr = eye(6);
param.WAMV.Dr = zeros(6,3);

% Check controllability
COr = ctrb(param.WAMV.Ar,param.WAMV.Br);
assert(rank(COr) == min(size(COr)),'System not controllable!')

%% LQR control design
Q = diag([100,100,1,1,1,100]);
R = diag([1,1,1]);

Sysr = ss(param.WAMV.Ar, param.WAMV.Br, param.WAMV.Cr, param.WAMV.Dr);
[param.WAMV.K,S,E] = lqr(Sysr,Q,R,zeros(6,3));

% Initial and final conditions
param.WAMV.etai = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
param.WAMV.etad = [20; 0; 0; 0; 0; 0];
param.WAMV.etad3 = param.WAMV.etad(param.WAMV.dofIdx); % Get desired eta for the 3 dof of choice

%%
sim('WAMVmodel');

%% Test-drive the thruster model and control allocation functions

% Run unit tests
%testResults = runtests('thrusterTest.m');
%assert(all([testResults(:).Passed]))