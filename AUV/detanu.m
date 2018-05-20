function dx = detanu(in)

global param

tau3 = in(1:5);  % X, Y, Z, N
eta3 = in(6:10);  % N, E, D, psi
nu3  = in(11:15);  % u, v, w, p, r
vc  = in(16:18);  % irrotational underwater current disturbances
etabar = in(18:22);

nu3r = nu3 + [vc;0;0];
eta3 = eta3 + etabar;
% Pad eta with zeros
eta = zeros(6,1);
eta(param.dofIdx) = eta3;            % N, E, D, phi, theta, psi

% Obtain kinematic transformation for DOF of interest
J = eulerKinematicTransformation(eta);
J3 = J(param.dofIdx,param.dofIdx);

omegaBNb = [0; in(14); in(15)];
SomegaBNb = skew(omegaBNb);
CRB = [param.m*SomegaBNb   -param.m*SomegaBNb*param.SrCBb;
       param.m*param.SrCBb*SomegaBNb    -skew(param.IBb*omegaBNb)];
CRB3 = CRB(param.dofIdx,param.dofIdx);

MA = diag([param.Xudot param.Yvdot param.Yvdot 0.3864 0.3864]);


CA = [0                                             0        0      0       param.Yvdot*nu3r(2)+param.Yrdot*nu3r(3);
      0                                             0        0      0       -param.Xudot*nu3r(1);
      0                                             0        0      0       -param.Xudot*nu3r(1);
      0                                             0        0      0         param.Xudot*nu3r(1) + param.Xudot*nu3r(1)
      -param.Yvdot*nu3r(2)-param.Yrdot*nu3r(3)     param.Xudot*nu3r(1)          param.Xudot*nu3r(1) -param.Xudot*nu3r(1) - param.Xudot*nu3r(1)   0];
  
Dm = diag([-param.Xu-param.Xauu*abs(nu3r(1)),-param.Yv-param.Yavv*abs(nu3r(2)),-param.Yv-param.Yavv*abs(nu3r(2)),-param.Nr-param.Narr*abs(nu3r(3)),-param.Nr-param.Narr*abs(nu3r(3))]);
                

NB = [0;0;0.2*9.81;0;0];      % Net bouyancy
deta3 = J3*nu3;
% dnu3 = inv(param.MRB3+MA)*(tau3 - (CRB3*nu3 + CA*nu3r) - Dm*nu3r + NB);
dnu3 = inv(param.MRB5+MA)*(tau3 - CRB3*nu3 - Dm*nu3r + NB);


dx = [deta3;dnu3];
