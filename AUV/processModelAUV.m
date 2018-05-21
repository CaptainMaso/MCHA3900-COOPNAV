function [xnext,A,SQ] = processModelAUV(x,u,pauv)
%#codegen

eta6 = x(1:6);  % N, E, D, roll, psi
nu6  = x(7:12);  % u, v, w, p, r
J = eulerKinematicTransformation(eta5);
u = nu6(1); v = nu6(2); w = nu6(3); p = nu6(4); q = nu6(5); r = nu6(6);

CRB = [
[                 0,                         -pauv.pauv.m*r,               pauv.pauv.m*q, pauv.pauv.m*q*pauv.pauv.yg + pauv.pauv.m*r*pauv.pauv.zg,         -pauv.pauv.m*q*pauv.pauv.xg,         -pauv.pauv.m*r*pauv.pauv.xg];
[               pauv.pauv.m*r,                       0,              -pauv.pauv.m*p,         -pauv.pauv.m*p*pauv.pauv.yg, pauv.pauv.m*p*pauv.pauv.xg + pauv.pauv.m*r*pauv.pauv.zg,         -pauv.pauv.m*r*pauv.pauv.yg];
[              -pauv.pauv.m*q,                      pauv.pauv.m*p,                 0,         -pauv.pauv.m*p*pauv.pauv.zg,         -pauv.pauv.m*q*pauv.pauv.zg, pauv.pauv.m*p*pauv.pauv.xg + pauv.pauv.m*q*pauv.pauv.yg];
[ - pauv.pauv.m*q*pauv.pauv.yg - pauv.pauv.m*r*pauv.pauv.zg,        pauv.pauv.m*p*pauv.pauv.yg,            pauv.pauv.m*p*pauv.pauv.zg,               0,          pauv.IBbz*r,         -pauv.IBby*q];
[            pauv.pauv.m*q*pauv.pauv.xg,          - pauv.pauv.m*p*pauv.pauv.xg - pauv.pauv.m*r*pauv.pauv.zg,            pauv.pauv.m*q*pauv.pauv.zg,         -pauv.IBbz*r,               0,          pauv.IBbx*p];
[            pauv.pauv.m*r*pauv.pauv.xg,            pauv.pauv.m*r*pauv.pauv.yg, - pauv.pauv.m*p*pauv.pauv.xg - pauv.pauv.m*q*pauv.pauv.yg,          pauv.IBby*q,         -pauv.IBbx*p,               0];
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

deta6 = J*nu6;
dnu6 = inv(pauv.MRB+MA)*(-(CRB+CA)*nu6 - Dm*nu6);

% This is the Euler discretisation
dx = [deta6;dnu6];
xnext = [eta6;nu6] + dt*dx;

% This is the linearisation about x     ew gross
ProcessJacobian = [ 
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[ (pauv.Xu + pauv.Xauu*abs(u) + pauv.Xauu*conj(u)*sign(u))/(pauv.Xudot + pauv.m),                     (pauv.m*r - pauv.Yvdot*conj(r))/(pauv.Xudot + pauv.m),                    -(pauv.m*q - pauv.Zwdot*conj(q))/(pauv.Xudot + pauv.m),                                                         -(pauv.m*q*pauv.yg + pauv.m*r*pauv.zg)/(pauv.Xudot + pauv.m),               (pauv.Zwdot*w - pauv.m*conj(w) + pauv.m*pauv.xg*conj(q) - pauv.m*pauv.yg*conj(p) + pauv.m*q*pauv.xg)/(pauv.Xudot + pauv.m),               (pauv.m*conj(v) - pauv.Yvdot*v + pauv.m*pauv.xg*conj(r) - pauv.m*pauv.zg*conj(p) + pauv.m*r*pauv.xg)/(pauv.Xudot + pauv.m), 0, 0, 0, 0, 0, 0];
[                    -(pauv.m*r - pauv.Xudot*conj(r))/(pauv.Yvdot + pauv.m), (pauv.Yv + pauv.Yavv*abs(v) + pauv.Yavv*conj(v)*sign(v))/(pauv.Yvdot + pauv.m),                     (pauv.m*p - pauv.Zwdot*conj(p))/(pauv.Yvdot + pauv.m),               (pauv.m*conj(w) - pauv.Zwdot*w - pauv.m*pauv.xg*conj(q) + pauv.m*pauv.yg*conj(p) + pauv.m*p*pauv.yg)/(pauv.Yvdot + pauv.m),                                                         -(pauv.m*p*pauv.xg + pauv.m*r*pauv.zg)/(pauv.Yvdot + pauv.m),               (pauv.Xudot*u - pauv.m*conj(u) + pauv.m*pauv.yg*conj(r) - pauv.m*pauv.zg*conj(q) + pauv.m*r*pauv.yg)/(pauv.Yvdot + pauv.m), 0, 0, 0, 0, 0, 0];
[                     (pauv.m*q - pauv.Xudot*conj(q))/(pauv.Zwdot + pauv.m),                    -(pauv.m*p - pauv.Yvdot*conj(p))/(pauv.Zwdot + pauv.m), (pauv.Zw + pauv.Zaww*abs(w) + pauv.Zaww*conj(w)*sign(w))/(pauv.Zwdot + pauv.m),               (pauv.Yvdot*v - pauv.m*conj(v) - pauv.m*pauv.xg*conj(r) + pauv.m*pauv.zg*conj(p) + pauv.m*p*pauv.zg)/(pauv.Zwdot + pauv.m),               (pauv.m*conj(u) - pauv.Xudot*u - pauv.m*pauv.yg*conj(r) + pauv.m*pauv.zg*conj(q) + pauv.m*q*pauv.zg)/(pauv.Zwdot + pauv.m),                                                         -(pauv.m*p*pauv.xg + pauv.m*q*pauv.yg)/(pauv.Zwdot + pauv.m), 0, 0, 0, 0, 0, 0];
[                      (pauv.m*q*pauv.yg + pauv.m*r*pauv.zg)/(pauv.IBbx + pauv.Kpdot),    -(Yvdot*conj(w) - pauv.Zwdot*w + pauv.m*p*pauv.yg)/(pauv.IBbx + pauv.Kpdot),    -(pauv.Yvdot*v - pauv.Zwdot*conj(v) + pauv.m*p*pauv.zg)/(pauv.IBbx + pauv.Kpdot), (pauv.Kp + pauv.Kapp*abs(p) - pauv.m*pauv.yg*conj(v) - pauv.m*pauv.zg*conj(w) + pauv.Kapp*conj(p)*sign(p))/(pauv.IBbx + pauv.Kpdot),              (pauv.Nrdot*r - pauv.IBbz*r + conj(r)*(pauv.IBby - pauv.Mqdot) + pauv.m*pauv.yg*conj(u))/(pauv.IBbx + pauv.Kpdot),              (pauv.IBby*q - pauv.Mqdot*q - conj(q)*(pauv.IBbz - pauv.Nrdot) + pauv.m*pauv.zg*conj(u))/(pauv.IBbx + pauv.Kpdot), 0, 0, 0, 0, 0, 0];
[    -(pauv.Zwdot*w - pauv.Xudot*conj(w) + pauv.m*q*pauv.xg)/(pauv.IBby + pauv.Mqdot),                      (pauv.m*p*pauv.xg + pauv.m*r*pauv.zg)/(pauv.IBby + pauv.Mqdot),    -(pauv.Zwdot*conj(u) - pauv.Xudot*u + pauv.m*q*pauv.zg)/(pauv.IBby + pauv.Mqdot),              (pauv.IBbz*r - pauv.Nrdot*r - conj(r)*(pauv.IBbx - pauv.Kpdot) + pauv.m*pauv.xg*conj(v))/(pauv.IBby + pauv.Mqdot), (pauv.Mq + pauv.Maqq*abs(q) - pauv.m*pauv.xg*conj(u) - pauv.m*pauv.zg*conj(w) + pauv.Maqq*conj(q)*sign(q))/(pauv.IBby + pauv.Mqdot),              (pauv.Kpdot*p - pauv.IBbx*p + conj(p)*(pauv.IBbz - pauv.Nrdot) + pauv.m*pauv.zg*conj(v))/(pauv.IBby + pauv.Mqdot), 0, 0, 0, 0, 0, 0];
[    -(pauv.Xudot*conj(v) - pauv.Yvdot*v + pauv.m*r*pauv.xg)/(pauv.IBbz + pauv.Nrdot),    -(pauv.Xudot*u - pauv.Yvdot*conj(u) + pauv.m*r*pauv.yg)/(pauv.IBbz + pauv.Nrdot),                      (pauv.m*p*pauv.xg + pauv.m*q*pauv.yg)/(pauv.IBbz + pauv.Nrdot),              (pauv.Mqdot*q - pauv.IBby*q + conj(q)*(pauv.IBbx - pauv.Kpdot) + pauv.m*pauv.xg*conj(w))/(pauv.IBbz + pauv.Nrdot),              (pauv.IBbx*p - pauv.Kpdot*p - conj(p)*(pauv.IBby - pauv.Mqdot) + pauv.m*pauv.yg*conj(w))/(pauv.IBbz + pauv.Nrdot), (pauv.Nr + pauv.Narr*abs(r) - pauv.m*pauv.xg*conj(u) - pauv.m*pauv.yg*conj(v) + pauv.Narr*conj(r)*sign(r))/(pauv.IBbz + pauv.Nrdot), 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0];
[                                                     0,                                                     0,                                                     0,                                                                                      0,                                                                                      0,                                                                                      0, 0, 0, 0, 0, 0, 0]
 ];
A = eye(18) + dt*(ProcessJacobian);

% Square root of process noise covariance
SQ = diag(ones(18,1));