function dx = auv_detanu(in)

global param
pauv = param.AUV;

tau5 = in(1:5);  % X, Y, Z, N
eta5 = in(6:10);  % N, E, D, psi
nu5  = in(11:15);  % u, v, w, p, r
vc  = in(16:18);  % irrotational underwater current disturbances
etabar = in(18:22); % pose disturbances

nu5r = nu5 + [vc;0;0];
eta5 = eta5 + etabar;
% Pad eta with zeros
eta = zeros(6,1);
eta(pauv.dofIdx) = eta5;            % N, E, D, phi, theta, psi

% Obtain kinematic transformation for DOF of interest
J = eulerKinematicTransformation(eta);
J5 = J(pauv.dofIdx,pauv.dofIdx);

omegaBNb = [in(14); 0;  in(15)];
SomegaBNb = skew(omegaBNb);

CRB = [pauv.m*SomegaBNb             -pauv.m*SomegaBNb*pauv.SrCBb;
       pauv.m*pauv.SrCBb*SomegaBNb    -skew(pauv.IBb*omegaBNb)];
   
CRB5 = CRB(pauv.dofIdx,pauv.dofIdx);

MA = diag([pauv.Xudot, pauv.Yvdot, pauv.Zwdot, pauv.Kpdot, pauv.Nrdot]);

u = nu5r(1);    v = nu5r(2);    w = nu5r(3);

CA = [[      0,            0,                         0,                    0,              pauv.Yvdot*v];
    [        0,            0,                         0,             pauv.Zwdot*w,          -pauv.Xudot*u];
    [        0,            0,                         0,             -pauv.Yvdot*v,                  0];
    [        0,     -pauv.Zwdot*w,              pauv.Yvdot*v,               0,                       0];
    [ -pauv.Yvdot*v,  pauv.Xudot*u,                  0,                     0                        0] ];

Dm = diag([-pauv.Xu-pauv.Xauu*abs(nu5r(1)),-pauv.Yv-pauv.Yavv*abs(nu5r(2)),-pauv.Zw-pauv.Zaww*abs(nu5r(3)),-pauv.Kp-pauv.Kapp*abs(nu5r(4)),-pauv.Nr-pauv.Narr*abs(nu5r(5))]);                

NB = [0;0;0.2*9.81;0;0];      % Net bouyancy
deta5 = J5*nu5r;
dnu5 = inv(pauv.MRB5+MA)*(tau5 -(CRB5+CA)*nu5r - Dm*nu5r + NB);


dx = [deta5;dnu5];
