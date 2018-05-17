syms N E D Phi Theta Psi u v w p q r X Y Z K M N m Ix Iy Iz
assume( [N E D Phi Theta Psi u v w p q r X Y Z K M N Ix Iy Iz], 'real');

global param
param.m = m;
param.SrCBb = skew([0;0;0]);
param.IBb = diag([Ix Iy Iz]);
param.MRB = [param.m*eye(3), -param.m*param.SrCBb;
             param.m*param.SrCBb, param.IBb];

x = [N E D Phi Theta Psi u v w p q r]';
Tau = [X Y Z K M N]';

dX = detanu([Tau;x]);
