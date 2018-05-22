function dx = auv_detanu(in)

global param
pauv = param.AUV;

tau6 = in(1:6);  % X, Y, Z, N
eta6 = in(7:12);  % N, E, D, psi
nu6  = in(13:18);  % u, v, w, p, r
u = nu6(1); v = nu6(2); w = nu6(3); p = nu6(4); q = nu6(5); r = nu6(6);

% Obtain kinematic transformation for DOF of interest
J = eulerKinematicTransformation(eta6);

CRB = [
[                 0,                         -pauv.m*r,               pauv.m*q, pauv.m*q*pauv.yg + pauv.m*r*pauv.zg,         -pauv.m*q*pauv.xg,         -pauv.m*r*pauv.xg];
[               pauv.m*r,                       0,              -pauv.m*p,         -pauv.m*p*pauv.yg, pauv.m*p*pauv.xg + pauv.m*r*pauv.zg,         -pauv.m*r*pauv.yg];
[              -pauv.m*q,                      pauv.m*p,                 0,         -pauv.m*p*pauv.zg,         -pauv.m*q*pauv.zg, pauv.m*p*pauv.xg + pauv.m*q*pauv.yg];
[ - pauv.m*q*pauv.yg - pauv.m*r*pauv.zg,        pauv.m*p*pauv.yg,            pauv.m*p*pauv.zg,               0,          pauv.IBbz*r,         -pauv.IBby*q];
[            pauv.m*q*pauv.xg,          - pauv.m*p*pauv.xg - pauv.m*r*pauv.zg,            pauv.m*q*pauv.zg,         -pauv.IBbz*r,               0,          pauv.IBbx*p];
[            pauv.m*r*pauv.xg,            pauv.m*r*pauv.yg, - pauv.m*p*pauv.xg - pauv.m*q*pauv.yg,          pauv.IBby*q,         -pauv.IBbx*p,               0];
 ];
 
MA = diag([pauv.Xudot, pauv.Yvdot, pauv.Zwdot, pauv.Kpdot, pauv.Mqdot, pauv.Nrdot]);

CA = [
[        0,               0,             0,                0,       -pauv.Zwdot*w,          pauv.Yvdot*v];
[        0,              0,              0,         pauv.Zwdot*w,        0,              -pauv.Xudot*u];
[        0,               0,             0,         -pauv.Yvdot*v,  pauv.Xudot*u,               0];
[        0,         -pauv.Zwdot*w,  pauv.Yvdot*v,        0,         -pauv.Nrdot*r,              pauv.Mqdot*q];
[  pauv.Zwdot*w,        0,          -pauv.Xudot*u,   Npauv.rdot*r,        0,            -pauv.Kpdot*p];
[ -pauv.Yvdot*v,     pauv.Xudot*u,        0,        -pauv.Mqdot*q,  pauv.Kpdot*p,            0]
];
 
Dm = diag([-pauv.Xu-pauv.Xauu*abs(u),-pauv.Yv-pauv.Yavv*abs(v),-pauv.Zw-pauv.Zaww*abs(w),-pauv.Kp-pauv.Kapp*abs(p),-pauv.Mq-pauv.Maqq*abs(q),-pauv.Nr-pauv.Narr*abs(r)]);

NB = [0;0;0.2*9.81;0;0];      % Net bouyancy
deta6 = J*nu6;


dnu6 = inv(pauv.MRB+MA)*(tau6 -(CRB+CA)*nu6 - Dm*nu6 + NB);


dx = [deta6;dnu6];
