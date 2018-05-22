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

TauPsi  = -param.WAMV.Kpsi*ePsi - param.WAMV.Kdpsi*nu(3);% - param.WAMV.Kxpsi*norm(eNE)*atan2(eN, eE);

RNB = [cos(psi) -sin(psi); sin(psi) cos(psi)];

rWDn = RNB'*rWDn;
TauNE   = -param.WAMV.Kx*rWDn - param.WAMV.Kdx*nu(1:2);

TauBstar = [TauNE;TauPsi];