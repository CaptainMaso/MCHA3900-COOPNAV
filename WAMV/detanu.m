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
