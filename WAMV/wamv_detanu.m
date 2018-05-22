function dx = wamv_detanu(in)
global param

tau = in(1:6);  % X, Y, Z, K, M, N
eta = in(7:12);  % N, E, D, phi, theta, psi
nu  = in(13:18);  % u, v, w, p, q, r

J = eulerKinematicTransformation(eta);
omegaBNb = [0;0;nu(6)];
SomegaBNb = skew(omegaBNb);

u = nu(1); v = nu(2); w = nu(3); p = nu(4); q = nu(5); r = nu(6); 
CRB = [                 param.WAMV.m*SomegaBNb, -param.WAMV.m*SomegaBNb*param.WAMV.SrCBb;
       param.WAMV.m*param.WAMV.SrCBb*SomegaBNb,          -skew(param.WAMV.IBb*omegaBNb)];
   
% CA = [           0,                  0,                 0,                 0, -param.WAMV.Zdw*w,   param.WAMV.Ydv*v;
%                  0,                  0,                 0,  param.WAMV.Zdw*w,                 0,  -param.WAMV.Xdu*u;
%                  0,                  0,                 0, -param.WAMV.Ydv*v,  param.WAMV.Xdu*u,                  0;
%                  0,  -param.WAMV.Zdw*w,  param.WAMV.Ydv*v,                 0, -param.WAMV.Ndr*r,   param.WAMV.Mdq*q;
%   param.WAMV.Zdw*w,                  0, -param.WAMV.Xdu*u,  param.WAMV.Ndr*r,                 0,  -param.WAMV.Kdp*p;
%  -param.WAMV.Ydv*v,   param.WAMV.Xdu*u,                 0, -param.WAMV.Mdq*q,  param.WAMV.Kdp*p,                  0];
%  
%Dnon = -diag([param.WAMV.Xuu*abs(u),param.WAMV.Yvv*abs(v),param.WAMV.Zww*abs(w)*0,param.WAMV.Kpp*abs(p),param.WAMV.Mqq*abs(q),param.WAMV.Nrr*abs(r)]);  
%D = param.WAMV.Dlin + Dnon;
  
NB = [0;0;4171.212;0;0;0];

MA = diag([-param.WAMV.Xdu, -param.WAMV.Ydv, 0, 0, 0 -param.WAMV.Ndr]);
MA(6,2) = -param.WAMV.Ydr;
MA(2,6) = -param.WAMV.Ydr;

CA = zeros(6,6);
CA(1,6) = param.WAMV.Ydv*v + param.WAMV.Ydr*v;
CA(2,6) = -param.WAMV.Xdu*u;
CA(6,1) = -param.WAMV.Ydv*v-param.WAMV.Ydr*r;
CA(6,2) = param.WAMV.Xdu*u;

D = diag([-param.WAMV.Xu-param.WAMV.Xuu*abs(u),-param.WAMV.Yv-param.WAMV.Yvv*abs(v),0,0,0,-param.WAMV.Nr-param.WAMV.Nrr*abs(r)]);
  
deta = J*nu;
dnu = inv(param.WAMV.MRB+MA)*(tau -(CRB+CA)*nu - D*nu);
%dnu = inv(param.WAMV.MRB)*(tau - CRB*nu);
dx = [deta; dnu];
