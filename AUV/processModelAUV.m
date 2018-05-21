function [xnext,A,SQ] = processModelAUV(x,u,pauv)
%#codegen

eta5 = x(1:6);  % N, E, D, roll, psi
nu5  = x(7:12);  % u, v, w, p, r
J = eulerKinematicTransformation(eta5);
J5 = J(pauv.dofIdx,pauv.dofIdx);

omegaBNb = [in(14); 0;  in(15)];
SomegaBNb = skew(omegaBNb);

CRB = [pauv.m*SomegaBNb             -pauv.m*SomegaBNb*pauv.SrCBb;
       pauv.m*pauv.SrCBb*SomegaBNb    -skew(pauv.IBb*omegaBNb)];
   
CRB5 = CRB(pauv.dofIdx,pauv.dofIdx);

MA = diag([pauv.Xudot, pauv.Yvdot, pauv.Zwdot, pauv.Kpdot, pauv.Nrdot]);


CA = [[      0,            0,                         0,                    0,              pauv.Yvdot*v];
    [        0,            0,                         0,             pauv.Zwdot*w,          -pauv.Xudot*u];
    [        0,            0,                         0,             -pauv.Yvdot*v,                  0];
    [        0,     -pauv.Zwdot*w,              pauv.Yvdot*v,               0,                       0];
    [ -pauv.Yvdot*v,  pauv.Xudot*u,                  0,                     0                        0] ];

Dm = diag([-pauv.Xu-pauv.Xauu*abs(nu5(1)),-pauv.Yv-pauv.Yavv*abs(nu5(2)),-pauv.Zw-pauv.Zaww*abs(nu5(3)),-pauv.Kp-pauv.Kapp*abs(nu5(4)),-pauv.Nr-pauv.Narr*abs(nu5(6))]);
                

NB = [0;0;0.2*9.81;0;0];      % Net bouyancy
deta5 = J5*nu5;
dnu5 = inv(pauv.MRB5+MA)*(-(CRB5+CA)*nu5 - Dm*nu5 + NB);


% This is the Euler discretisation
dx = [deta5;dnu5];
xnext = [eta5;nu5] + dt*dx;

% This is the linearisation about x     ew gross

AUVCJacobian = [0,	pauv.Yvdot*r/(pauv.Xudot + pauv.m),	0,	(pauv.m*r*pauv.zg)/(pauv.Xudot + pauv.m),	(pauv.Yvdot*v - 2*pauv.m*r*pauv.xg)/(pauv.Xudot + pauv.m)-(pauv.m*v)/(pauv.Xudot + pauv.m)+(pauv.m*p*pauv.zg)/(pauv.Xudot + pauv.m);
-r*(pauv.Xudot)/(pauv.Yvdot+pauv.m)+(pauv.m*r)/(pauv.Yvdot + pauv.m),	0,	p*pauv.Zwdot/(pauv.Yvdot + pauv.m)- (pauv.m*p)/(pauv.Yvdot + pauv.m),	(pauv.Zwdot*w - 2*pauv.m*p*pauv.yg)/(pauv.Yvdot + pauv.m)- (pauv.m*w)/(pauv.Yvdot + pauv.m),		- (pauv.Xudot*u + 2*pauv.m*r*pauv.yg)/(pauv.Yvdot + pauv.m)  + (pauv.m*u)/(pauv.Yvdot + pauv.m);
	0,	pauv.m*p/(pauv.Zwdot + pauv.m) - p*pauv.Yvdot/(pauv.Zwdot + pauv.m),		0,	pauv.m*v/(pauv.Zwdot + pauv.m) - (pauv.Yvdot*v + 2*pauv.m*p*pauv.zg)/(pauv.Zwdot + pauv.m) + (pauv.m*r*pauv.xg)/(pauv.Zwdot + pauv.m),	pauv.m*p*pauv.xg/(pauv.Zwdot + pauv.m);
-(pauv.m*r*pauv.zg)/(pauv.IBbx + pauv.Kpdot),	w*pauv.Yvdot/(pauv.IBbx + pauv.Kpdot) - (pauv.Zwdot*w - pauv.m*p*pauv.yg)/(pauv.IBbx + pauv.Kpdot),	(pauv.Yvdot*v + pauv.m*p*pauv.zg)/(pauv.IBbx + pauv.Kpdot)-v*pauv.Zwdot/(pauv.IBbx + pauv.Kpdot),	w*pauv.m*pauv.zg/(pauv.IBbx + pauv.Kpdot)-v*pauv.m*pauv.yg/(pauv.IBbx + pauv.Kpdot),	 -(pauv.m*u*pauv.zg)/(pauv.IBbx + pauv.Kpdot);
v*pauv.Xudot/(pauv.IBbz + pauv.Nrdot)-(pauv.Yvdot*v - pauv.m*r*pauv.xg)/(pauv.IBbz + pauv.Nrdot),	(pauv.Xudot*u + pauv.m*r*pauv.yg)/(pauv.IBbz + pauv.Nrdot)-u*pauv.Yvdot/(pauv.IBbz + pauv.Nrdot),	- pauv.m*p*pauv.xg/(pauv.IBbz + pauv.Nrdot),	 - pauv.m*w*pauv.xg/(pauv.IBbz + pauv.Nrdot),	v*pauv.m*pauv.yg/(pauv.IBbz + pauv.Nrdot) - u*pauv.m*pauv.xg/(pauv.IBbz + pauv.Nrdot)];									

AUVDJacobian = diag([
    -(pauv.Xu + 2*pauv.Xauu*u)/(pauv.Xudot + pauv.m);
    -(pauv.Yv + 2*pauv.Yavv*v)/(pauv.Yvdot + pauv.m);
    -(pauv.Zw + 2*pauv.Zaww*w)/(pauv.Zwdot + pauv.m);
    -(pauv.Kp + 2*pauv.Kapp*p)/(pauv.IBbx + pauv.Kpdot);
    -(pauv.Nr + 2*pauv.Narr*r)/(pauv.IBbz + pauv.Nrdot)	]);

A = eye(10) + dt*([AUVCJacobian+AUVDJacobian, zeros(5,5); zeros(5,10)]);

% Square root of process noise covariance
SQ = diag(ones(10,1));