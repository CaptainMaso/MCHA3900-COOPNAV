function TauBstar = wamv_CTRL(in)

global param

ref = in(1:3);
X   = in(4:9);

eta = X(1:3);
nu  = X(4:6);

psi = eta(3);

rWDn = [eta(1) - ref(1);
        eta(2) - ref(2)];
ePsi = psi - ref(3);

beta = atan2(rWDn(2),-rWDn(1));
gamma = psi - beta;

r = -norm(rWDn);
eNb = r*cos(gamma);
eEb = r*sin(gamma);

TauPsi  = -param.WAMV.Kpsi*ePsi - param.WAMV.Kdpsi*nu(3) - param.WAMV.Kdxpsi*r*gamma;% - param.WAMV.Kxpsi*norm(eNE)*atan2(eN, eE);
TauN   = -param.WAMV.KN*eNb - param.WAMV.KdN*nu(1);
TauE   = -param.WAMV.KE*eEb - param.WAMV.KdE*nu(2);

TauBstar = [TauN;TauE;TauPsi];