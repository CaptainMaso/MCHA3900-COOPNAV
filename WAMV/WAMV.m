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

param.WAMV.DRB = diag([10, 10000, 100, 100, 100, 10]);
param.WAMV.KRB = diag([0, 0, 1000, 100, 100, 0]);

% Centre of gravity
param.WAMV.xg    = 0;                  
param.WAMV.yg    = 0;
param.WAMV.zg    = 0;
param.WAMV.rCBb  = [param.WAMV.xg;param.WAMV.yg;param.WAMV.zg];
param.WAMV.SrCBb = skew(param.WAMV.rCBb);

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

param.WAMV.Kx = 10;
param.WAMV.Kdx = 40;
param.WAMV.Kpsi = 10;
param.WAMV.Kdpsi = 40;
param.WAMV.Kxpsi = 1;

% Initial and final conditions
param.WAMV.etai = [0; 0; 0; 0; 0; pi/4; zeros(6,1)];
param.WAMV.etad = [0; 5; 0; 0; 0; pi/4];
param.WAMV.etad3 = param.WAMV.etad(param.WAMV.dofIdx); % Get desired eta for the 3 dof of choice

%%
sim('WAMVmodel');

%% Test-drive the thruster model and control allocation functions

% Run unit tests
%testResults = runtests('thrusterTest.m');
%assert(all([testResults(:).Passed]))