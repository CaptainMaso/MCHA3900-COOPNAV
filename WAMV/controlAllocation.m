function u = controlAllocation(tau3)

% Inputs:
%   tau3   : column vector of generalised forces
%   tau3(1): Surge force [N]
%   tau3(2): Sway force [N]
%   tau3(3): Yaw torque [Nm]

% Outputs:
%   u   : column vector of actuator forces
%   u(1): aft-starboard thruster force [N]
%   u(2): aft-port thruster force [N]
%   u(3): fore-port thruster force [N]
%   u(4): fore-starboard thruster force [N]

B = actuatorConfiguration();
u = pinv(B)*tau3;