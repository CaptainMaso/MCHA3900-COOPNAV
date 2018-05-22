function dX = quad_detanu(in)
global param
X = in(7:18);
TauB = in(1:6);      % X, Y, Z, K, M, N
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

Taun = -param.QUAD.DRB*deta;

Taun = Taun + [0;0;param.QUAD.m*param.g; 0; 0; 0];

TauB = TauB + [Rnb'*Taun(1:3);Taun(4:6)];

% Matrix Assembly
A = [zeros(6), Jeta; zeros(6), -(param.QUAD.MRB\CRBnu)];
BTau = [zeros(6,1); param.QUAD.MRB\TauB];
       
dX = A*X + BTau;
