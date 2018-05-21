%% UUV model in the horizontal plane
global param

% Indices for degrees of freedom of interest
param.AUV.dofIdx = [1 2 3 4 6]; % N, E, D, psi(yaw)

% Mass [kg]
param.AUV.m = 10.17; 

% Inertia
param.AUV.IBb   = diag([0.139 0.171 0.212]);

% param.AUV.IBb   = [0.98657 0.06241 0.15094;
%                -0.06085 0.99804 -0.01492;
%                -0.15158 0.00554 0.98843];

% Centre of gravity
param.AUV.xg    = 0;                  
param.AUV.yg    = 0;
param.AUV.zg    = 0;
param.AUV.rCBb  = [param.AUV.xg;param.AUV.yg;param.AUV.zg];
param.AUV.SrCBb = skew(param.AUV.rCBb);

% Hydrodynamic parameters
param.AUV.Xudot = -16.75; param.AUV.Xu = -0.03;     param.AUV.Xauu = -0.05;
param.AUV.Yvdot = -0.032752; param.AUV.Yrdot = -16.75;  param.AUV.Yv = -0.1;    param.AUV.Yavv = -0.03;
param.AUV.Nrdot = -0.015;  param.AUV.Nr = -0.02;    param.AUV.Narr = 0;

% Generalised mass matrix (constant in body coordinates)
param.AUV.MRB = [param.AUV.m.*eye(3) -param.AUV.m*param.AUV.SrCBb;
             param.AUV.m*param.AUV.SrCBb param.AUV.IBb];
param.AUV.MRB5 = param.AUV.MRB(param.AUV.dofIdx,param.AUV.dofIdx);

MA = diag([param.AUV.Xudot param.AUV.Yvdot param.AUV.Yvdot 0.3864 0.3864]);

MRBA = MA + param.AUV.MRB5;

Dlin = diag([-param.AUV.Xauu -param.AUV.Yavv -param.AUV.Yavv -param.AUV.Narr -param.AUV.Narr]);
% Formulate linear surge-sway-yaw state-space model for control design
param.AUV.Ar = [zeros(5) eye(5);
            zeros(5) -inv(MRBA)*Dlin];
param.AUV.Br = [zeros(5); inv(MRBA)];

% outputs are position
param.AUV.Cr = [eye(5) zeros(5)];

param.AUV.Dr = 0;

param.AUV.MatrixAa = [param.AUV.Ar zeros(10,5);
            -param.AUV.Cr zeros(5)];
        
param.AUV.MatrixBa = [param.AUV.Br; zeros(5)];
param.AUV.MatrixCa = [param.AUV.Cr zeros(5,10)];

% Check controllability
COr = ctrb(param.AUV.MatrixAa,param.AUV.MatrixBa);

assert(rank(COr) == min(size(COr)),'System not controllable!')

%% LQR control design
Q = diag([1000 100 100000 100 1000 1 1 1 1 1    100 10 1000 100 100]);
% Q = diag([50 10 10 10 0.1 0.1 0.1 0.1     0.5 0.1 0.1 0.1]);

R = 2*eye(5);

Sysr = ss(param.AUV.Ar, param.AUV.Br, param.AUV.Cr, param.AUV.Dr);
[param.AUV.K,S,E] = lqi(Sysr,Q,R,zeros(15,5));

param.AUV.Kgain = [param.AUV.K(:,1:10) param.AUV.K(:,11:15)];

% Initial and final conditions
param.AUV.etai = [0; 0; 0; 0; 0; 0; 0; 0];
param.AUV.etad5 = [0; 0; 0; 0; 5]; % Get desired eta for the 3 dof of choice

%% check eigenvalues
eigABK = eig(param.AUV.MatrixAa - param.AUV.MatrixBa*param.AUV.K);
disturbances_eta = zeros(5,1);
sim('AUVsim')

