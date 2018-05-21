
%% UUV model in the horizontal plane
global param

% Indices for degrees of freedom of interest
param.dofIdx = [1 2 6]; % N, E, psi(yaw)

param.g = 9.81;

% Mass [kg]
param.m = 200; 

param.b = 2.44; % beam
param.l = 4.85; % length
param.h = 1.28; % height

% Inertia
param.r     = 0.5;                  % Yaw gyradius [m]
param.Ix = param.m*param.b*0.35;
param.Iy = param.m*param.l*0.35;
param.Iz    = param.m*param.r^2; 	% Yaw moment of inertia [kg m^2]
param.IBb   = diag([param.Ix,param.Iy,param.Iz]);

% Centre of gravity
param.xg    = 0.2;                  
param.yg    = 0;
param.zg    = 0.2;
param.rCBb  = [param.xg;param.yg;param.zg];
param.SrCBb = skew(param.rCBb);

% Generalised mass matrix (constant in body coordinates)
param.MRB = [param.m*eye(3), -param.m*param.SrCBb;
            param.m*param.SrCBb, param.IBb];        %from 3.79;
param.MRB3 = param.MRB(param.dofIdx,param.dofIdx);

% Formulate linear surge-sway-yaw state-space model for control design
param.Ar = [zeros(3), eye(3); zeros(3), -inv(param.MRB3)];
param.Br = [zeros(3); inv(param.MRB3)];
param.Cr = eye(6);
param.Dr = zeros(6,3);

% Check controllability
COr = ctrb(param.Ar,param.Br);
assert(rank(COr) == min(size(COr)),'System not controllable!')

%% LQR control design
Q = diag([0.01,0.1,0.1,1,0.1,1]);
R = eye(3).*0.5;

Sysr = ss(param.Ar, param.Br, param.Cr, param.Dr);
[K,S,E] = lqr(Sysr,Q,R,zeros(6,3));

% Initial and final conditions
etai = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
etad = [50; 0; 0; 0; 0; 0];
etad3 = etad(param.dofIdx); % Get desired eta for the 3 dof of choice

%%
sim('WAMVmodel')

%% Test-drive the thruster model and control allocation functions

% Run unit tests
%testResults = runtests('thrusterTest.m');
%assert(all([testResults(:).Passed]))



