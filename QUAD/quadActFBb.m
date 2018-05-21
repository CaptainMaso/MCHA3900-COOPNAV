function tau = quadActFBb(in)
% Converts actuator commands to body forces
global param

eta = in(1:6);
nu = in(7:12);

u = in(13:end);

Fb = [0;0;sum(u)];

TbB = [0, -param.QUAD.lprop,   0, param.QUAD.lprop;
             param.QUAD.lprop, 0, -param.QUAD.lprop, 0;
             -param.QUAD.cprop, param.QUAD.cprop, -param.QUAD.cprop, param.QUAD.cprop] *u;
  
tau = [Fb;TbB];