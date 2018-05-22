function etap3 = auv_stateTransformation(eta3)

global param

% Pad eta with zeros
eta = zeros(6,1);
eta(param.AUV.dofIdx) = eta3;            % N, E, D, phi, theta, psi

% Obtain kinematic transformation for DOF of interest
J = eulerKinematicTransformation(eta);
J3 = J(param.AUV.dofIdx,param.AUV.dofIdx);

etap3 = J3\eta3;