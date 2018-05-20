function invJ = eulerInvKinematicTransformation(eta)
%eulerKinematicTransformation    Obtain kinematic transformation from Euler angles
%   J = eulerKinematicTransformation(eta)
%   takes column vector of Euler angles and returns transformation J,
%   such that
%       deta = J*nu
%
%   See also eulerRotation, eulerKDE.

thetanb = eta(4:6);

Rnb = inveulerRotation(thetanb);
TK = inveulerKDE(thetanb);

invJ = [Rnb,zeros(3);zeros(3),TK];