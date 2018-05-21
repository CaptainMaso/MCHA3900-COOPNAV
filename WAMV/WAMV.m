%% UUV model in the horizontal plane
global param

% Indices for degrees of freedom of interest
param.WAMV.dofIdx = [1 2 6]; % N, E, psi(yaw)

param.g = 9.81;

% Mass [kg]
param.WAMV.m = 200; 

param.WAMV.b = 2.44; % beam
param.WAMV.l = 4.85; % length
param.WAMV.h = 1.28; % height

% Inertia
param.WAMV.r     = 0.5;                  % Yaw gyradius [m]
param.WAMV.Ix = param.WAMV.m*param.WAMV.b*0.35;
param.WAMV.Iy = param.WAMV.m*param.WAMV.l*0.35;
param.WAMV.Iz    = param.WAMV.m*param.WAMV.r^2; 	% Yaw moment of inertia [kg m^2]
param.WAMV.IBb   = diag([param.WAMV.Ix,param.WAMV.Iy,param.WAMV.Iz]);

% Centre of gravity
param.WAMV.xg    = 0.2;                  
param.WAMV.yg    = 0;
param.WAMV.zg    = 0.2;
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
Q = diag([0.001,0.1,0.1,1,0.1,1]);
R = eye(3);

Sysr = ss(param.WAMV.Ar, param.WAMV.Br, param.WAMV.Cr, param.WAMV.Dr);
[param.WAMV.K,S,E] = lqr(Sysr,Q,R,zeros(6,3));

% Initial and final conditions
param.WAMV.etai = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
param.WAMV.etad = [1; 1; 0; 0; 0; 0];
param.WAMV.etad3 = param.WAMV.etad(param.WAMV.dofIdx); % Get desired eta for the 3 dof of choice

%%
sim('WAMVmodel');

%% Test-drive the thruster model and control allocation functions

% Run unit tests
%testResults = runtests('thrusterTest.m');
%assert(all([testResults(:).Passed]))