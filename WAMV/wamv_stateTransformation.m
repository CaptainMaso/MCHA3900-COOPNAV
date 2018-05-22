function etap3 = wamv_stateTransformation(eta3)

global param

% Pad eta with zeros
eta = zeros(6,1);
eta(param.WAMV.dofIdx) = eta3;            % N, E, D, phi, theta, psi

% Obtain kinematic transformation for DOF of interest
J = eulerKinematicTransformation(eta);
J3 = J(param.WAMV.dofIdx,param.WAMV.dofIdx);

etap3 = J3\eta3;