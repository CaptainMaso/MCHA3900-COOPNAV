function TK = eulerKDE(thetanb)
%eulerKDE    Obtain transformation matrix for Euler angle kinematic differential equation
%   TK = eulerKDE(thetanb)
%   takes column vector of Euler angles and returns transformation TK,
%   such that
%       dthetanb = TK*omegaBNb
%
%   See also eulerRotation, eulerKinematicTransformation.

phi     = thetanb(1);   % Roll angle
theta   = thetanb(2);   % Pitch angle

TK = [ ...
    1, sin(phi)*tan(theta), cos(phi)*tan(theta); ...
    0, cos(phi),            -sin(phi); ...
    0, sin(phi)/cos(theta), cos(phi)/cos(theta) ...
    ];