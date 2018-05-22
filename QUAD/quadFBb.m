function TauB = quadFBb(X)
% Returns forces on the body of the vehicle in body coordinates (FBb)
% Tau = [X, Y, Z, K, M, N] or [Surge, Sway, Heave, Roll, Pitch, Yaw] Forces
global param

eta = X(1:6);    	% x, y, z, phi, theta, psi (eta)
nu = X(7:12);       % [u, v, w, p, q, r]'; (nu)    

% Calculations for body forces
deta = eulerKinematicTransformation(eta)*nu;
Rnb = eulerRotation(eta(4:6));

Taun = -param.QUAD.DRB*deta;

Taun = Taun + [0;0;param.QUAD.m*param.g; 0; 0; 0];


TauB = [Rnb'*Taun(1:3);Taun(4:6)];