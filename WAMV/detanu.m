<<<<<<< HEAD
function dx = detanu(in)
global param

tau = in(1:6);  % X, Y, Z, K, M, N
eta = in(7:12);  % N, E, D, phi, theta, psi
nu  = in(13:18);  % u, v, w, p, q, r

J = eulerKinematicTransformation(eta);
omegaBNb = [nu(4);nu(5);nu(6)];
SomegaBNb = skew(omegaBNb);

CRB = [param.m*SomegaBNb, -param.m*SomegaBNb*param.SrCBb;
       param.m*param.SrCBb*SomegaBNb, -skew(param.IBb*omegaBNb)];
   
Xdu = -126.9;
Xu  = -100.0;
Xuu = -269.5;
Ydv = -235.4;
Ydr = - 31.1;
Yv  = -526.0;
Yvv = -209.8;
Ndr = - 95.3;
Nr  = - 88.8;
Nrr = - 22.1;

Kpp = -180;   Kv  = -300;
Mqq = -180;   Mv  = -200;   
   
u = nu(1); v = nu(2); w = nu(3); p = nu(4); q = nu(5); r = nu(6);
CA = [                   0         0 0 0 0         Ydv*v+Ydr*r;
                         0         0 0 0 0              -Xdu*u;
                         0         0 0 0 0                   0;
                         0         0 0 0 0                   0;
                         0         0 0 0 0                   0;
              -Ydv*v-Ydr*r     Xdu*u 0 0 0                   0];
Dv = -diag([Xuu*abs(u), Yvv*abs(v), 0, Kpp*abs(p), Mqq*abs(q), Nrr*abs(r)]);
Dlin = -diag([Xu,Yv,0,Kv,Mv,Nr]);
D  = Dlin + Dv;  
  
MA = [       -Xdu        0  0 0 0            0;
                 0        -Ydv  0 0 0         -Ydr;
                 0           0  0 0 0            0;
                 0           0  0 0 0            0;
                 0           0  0 0 0            0;
                 0        -Ydr  0 0 0        -Ndr];

M = param.MRB + MA;
deta = J*nu;
dnu = (param.MRB)\(tau - CRB*nu);
%dnu  = (-M \(CRB+CA+D))*nu + M\tau;
dx = [deta; dnu];
=======
function dx = detanu(in)
global param

tau = in(1:6);  % X, Y, Z, K, M, N
eta = in(7:12);  % N, E, D, phi, theta, psi
nu  = in(13:18);  % u, v, w, p, q, r

J = eulerKinematicTransformation(eta);
omegaBNb = [nu(4);nu(5);nu(6)];
SomegaBNb = skew(omegaBNb);

CRB = [param.m*SomegaBNb, -param.m*SomegaBNb*param.SrCBb;
       param.m*param.SrCBb*SomegaBNb, -skew(param.IBb*omegaBNb)];
deta = J*nu;
dnu = param.MRB\(tau - CRB*nu);

dx = [deta; dnu];
>>>>>>> 104da6e860228230acae46531ee8b8e2325705c1
