function taustar = Controller(in)
global param

e3 = [0;0;1];

ref = in(1:4);
X   = in(5:16);

eta = X(1:6);
nu  = X(7:12);

xd = ref(1:3);

RBNb = eulerRotation(eta(4:6));
omegaBNb = nu(4:6);

x = eta(1:3);
ex = x-xd;

v = nu(1:3);
vd = [0,0,0]';
ev = v - vd;

ddxd = [0,0,0]';

omegad = [0,0,0]';
domegad = [0,0,0]';

b1d = [cos(ref(4)); sin(ref(4));0];

traj = -param.QUAD.kx*ex - param.QUAD.kv*ev - param.QUAD.m*param.g*e3 + param.QUAD.m*ddxd;

b3d = -traj/norm(traj);

b2d = cross(b3d,b1d)/norm(cross(b3d,b1d));

RBNbd = [cross(b2d,b3d), b2d, b3d];

er = unskew(RBNbd'*RBNb - RBNb'*RBNbd)./2;

eomega = omegaBNb - RBNb'*RBNbd*omegad;
 
FBd = dot(traj,(RBNb*e3));
MBd = -param.QUAD.kr*er - param.QUAD.kdr*eomega + cross(omegaBNb,param.QUAD.IBb*omegaBNb) - ...
      param.QUAD.IBb*(skew(omegaBNb)*RBNb'*RBNbd*omegad - RBNb'*RBNbd*domegad);

taustar = [FBd; MBd];