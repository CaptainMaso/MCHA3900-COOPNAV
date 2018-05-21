function R = inveulerRotation(thetanb)
%eulerRotation    Obtain rotation matrix from Euler angles
%   Rnb = eulerRotation(thetanb)
%   takes column vector of Euler angles and returns rotation Rnb,
%   such that
%       rn = Rnb*rb
%
%   See also eulerKDE, eulerKinematicTransformation.

phi     = thetanb(1);   % Roll angle
theta   = thetanb(2);   % Pitch angle
psi     = thetanb(3);   % Yaw angle

Rz = [ ...
     cos(psi), -sin(psi), 0; ...
     sin(psi),  cos(psi), 0; ...
            0,         0, 1 ...
    ];

Ry = [ ...
     cos(theta), 0, sin(theta); ...
              0, 1,          0; ...
    -sin(theta), 0, cos(theta) ...
    ];

Rx = [ ...
    1,        0,         0; ...
    0, cos(phi), -sin(phi); ...
    0, sin(phi),  cos(phi); ...
    ];

R = (Rz*Ry*Rx)';