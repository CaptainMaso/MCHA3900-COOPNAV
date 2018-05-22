function dX = quad_detanu(in)
global param

Taub = in(1:6);      % X, Y, Z, K, M, N
X = in(7:18);
eta = X(1:6);    	% x, x, z, phi, theta, psi (eta)
nu = X(7:12);       % [u, v, w, p, q, r]'; (nu)    

Jeta    = eulerKinematicTransformation(eta);

omegaBNb = nu(4:6);
SomegaBNb = skew(omegaBNb);
CRBnu     = [param.QUAD.m * SomegaBNb, -param.QUAD.m * SomegaBNb * param.QUAD.SrCBb;
           param.QUAD.m * param.QUAD.SrCBb * SomegaBNb, -skew(param.QUAD.IBb*omegaBNb)];

% % Body Forces
deta = Jeta*nu;
Rnb = eulerRotation(eta(4:6));

Taun = [0;0;param.QUAD.m*param.g; 0; 0; 0];
Taub = Taub + [Rnb'*Taun(1:3);Taun(4:6)];

dnu = param.QUAD.MRB\(Taub - CRBnu*nu - param.QUAD.DRB*nu);
       
dX = [deta;dnu];
