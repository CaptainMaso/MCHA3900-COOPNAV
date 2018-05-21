function dx = auv_detanu(in)

global param
pauv = param.AUV;

tau3 = in(1:5);  % X, Y, Z, N
eta3 = in(6:10);  % N, E, D, psi
nu3  = in(11:15);  % u, v, w, p, r
vc  = in(16:18);  % irrotational underwater current disturbances
etabar = in(18:22);

nu3r = nu3 + [vc;0;0];
eta3 = eta3 + etabar;
% Pad eta with zeros
eta = zeros(6,1);
eta(pauv.dofIdx) = eta3;            % N, E, D, phi, theta, psi

% Obtain kinematic transformation for DOF of interest
J = eulerKinematicTransformation(eta);
J3 = J(pauv.dofIdx,pauv.dofIdx);

omegaBNb = [0; in(14); in(15)];
SomegaBNb = skew(omegaBNb);
CRB = [pauv.m*SomegaBNb   -pauv.m*SomegaBNb*pauv.SrCBb;
       pauv.m*pauv.SrCBb*SomegaBNb    -skew(pauv.IBb*omegaBNb)];
CRB3 = CRB(pauv.dofIdx,pauv.dofIdx);

MA = diag([pauv.Xudot pauv.Yvdot pauv.Yvdot 0.3864 0.3864]);


CA = [0                                             0        0      0       pauv.Yvdot*nu3r(2)+pauv.Yrdot*nu3r(3);
      0                                             0        0      0       -pauv.Xudot*nu3r(1);
      0                                             0        0      0       -pauv.Xudot*nu3r(1);
      0                                             0        0      0         pauv.Xudot*nu3r(1) + pauv.Xudot*nu3r(1)
      -pauv.Yvdot*nu3r(2)-pauv.Yrdot*nu3r(3)     pauv.Xudot*nu3r(1)          pauv.Xudot*nu3r(1) -pauv.Xudot*nu3r(1) - pauv.Xudot*nu3r(1)   0];
  
Dm = diag([-pauv.Xu-pauv.Xauu*abs(nu3r(1)),-pauv.Yv-pauv.Yavv*abs(nu3r(2)),-pauv.Yv-pauv.Yavv*abs(nu3r(2)),-pauv.Nr-pauv.Narr*abs(nu3r(3)),-pauv.Nr-pauv.Narr*abs(nu3r(3))]);
                

NB = [0;0;0.2*9.81;0;0];      % Net bouyancy
deta3 = J3*nu3;
% dnu3 = inv(pauv.MRB3+MA)*(tau3 - (CRB3*nu3 + CA*nu3r) - Dm*nu3r + NB);
dnu3 = inv(pauv.MRB5+MA)*(tau3 - CRB3*nu3 - Dm*nu3r + NB);


dx = [deta3;dnu3];
