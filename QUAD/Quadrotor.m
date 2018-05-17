global param

% General Quad Parameters
param.QUAD.m = 4.34;         % Total Mass (kg)
param.QUAD.Ix = 0.0820;      % MoI in yz plane (kg.m^-4)
param.QUAD.Iy = 0.0845;      % MoI in xz plane (kg.m^-4)
param.QUAD.Iz = 0.1377;      % MoI in xy plane (kg.m^-4)

param.QUAD.lprop = 0.315;        % Distance from props to cog (m)
param.QUAD.cprop = 8.004e-4;     % Force to Moment constant (m: M/F)

% Forming Relevant Matrices
param.QUAD.rCBb = [0 0 0]';
param.QUAD.SrCBb = skew(param.QUAD.rCBb);

param.QUAD.IBb = diag([param.QUAD.Ix, param.QUAD.Iy, param.QUAD.Iz]);

param.QUAD.MRB = [param.QUAD.m*eye(3), -param.QUAD.m*param.QUAD.SrCBb;
             param.QUAD.m*param.QUAD.SrCBb, param.QUAD.IBb];
         
param.QUAD.DRB = diag([0.5, 0.5, 0.5, 0.05, 0.05, 0.05]);
          
% ------------- Initial States
param.QUAD.eta0 = [0,0,0,0,0,0]';
param.QUAD.nu0 = [0,0,0,0,0,0]';

param.QUAD.X0 = [param.QUAD.eta0;param.QUAD.nu0];

param.QUAD.ref = [5,5,-10,deg2rad(45)];
% ------------- Controller Parameters
param.QUAD.kx = 5;
param.QUAD.kv = 5;

param.QUAD.kr = 10;
param.QUAD.kdr = 1;
%% --------------- SIM QUAD
sim('quad');

%% --------------- Plot QUAD Output
