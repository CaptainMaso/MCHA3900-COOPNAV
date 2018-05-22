function dx = wamv_detanu(in)
global param

Taub = in(1:6);  % X, Y, Z, K, M, N
eta = in(7:12);  % N, E, D, phi, theta, psi
nu  = in(13:18);  % u, v, w, p, q, r

J = eulerKinematicTransformation(eta);
omegaBNb = [0;0;nu(6)];
SomegaBNb = skew(omegaBNb);

CRB = [                 param.WAMV.m*SomegaBNb, -param.WAMV.m*SomegaBNb*param.WAMV.SrCBb;
       param.WAMV.m*param.WAMV.SrCBb*SomegaBNb,          -skew(param.WAMV.IBb*omegaBNb)];

deta = J*nu;
dnu = (param.WAMV.MRB)\(Taub - (CRB+param.WAMV.DRB)*nu - param.WAMV.KRB*eta);

dx = [deta; dnu];
