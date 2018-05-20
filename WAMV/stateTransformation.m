function etap3 = stateTransformation(eta3)

global param

% Pad eta with zeros
eta = zeros(6,1);
eta(param.dofIdx) = eta3;            % N, E, D, phi, theta, psi

% Obtain kinematic transformation for DOF of interest
J = eulerKinematicTransformation(eta);
J3 = J(param.dofIdx,param.dofIdx);

etap3 = J3\eta3;