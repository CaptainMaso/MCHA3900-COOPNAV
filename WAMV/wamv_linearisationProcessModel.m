
syms m b l h g Rr Ix Iy Iz X Y Z K M Nt N E D phi theta psi u v w p q r Xu Xdu Xuu Yv Ydv Ydr Yvv Nr Ndr Nrr SrCBb IBb real

tau = [X, Y, Z, K, M, Nt]';  % X, Y, Z, K, M, N
eta = [N, E, D, phi, theta, psi]';  % N, E, D, phi, theta, psi
nu  = [u, v, w, p, q, r]';  % u, v, w, p, q, r
states = [eta; nu];

J = eulerKinematicTransformation(eta);
omegaBNb = [0;0;nu(6)];
SomegaBNb = skew(omegaBNb);
xg    = 0.2;                  
yg    = 0;
zg    = 0.2;
rCBb  = [xg;yg;zg];
SrCBb = skew(rCBb);
IBb   = diag([Ix,Iy,Iz]);

MRB = [m*eye(3), -m*SrCBb;
            m*SrCBb, IBb]; 

u = nu(1); v = nu(2); w = nu(3); p = nu(4); q = nu(5); r = nu(6); 
CRB = [                 m*SomegaBNb, -m*SomegaBNb*SrCBb;
       m*SrCBb*SomegaBNb,          -skew(IBb*omegaBNb)];
 

MA = diag([- Xdu, - Ydv, 0, 0, 0 - Ndr]);
MA(6,2) = - Ydr;
MA(2,6) = - Ydr;

CA = sym(zeros(6,6));
CA(1,6) =  Ydv*v +  Ydr*v;
CA(2,6) = - Xdu*u;
CA(6,1) = - Ydv*v- Ydr*r;
CA(6,2) =  Xdu*u;

D = diag([- Xu- Xuu*abs(u),- Yv- Yvv*abs(v),0,0,0,- Nr- Nrr*abs(r)]);
  
deta = J*nu;
dnu = inv( MRB+MA)*(tau -(CRB+CA)*nu - D*nu);

jacdeta  = jacobian(deta, states)
jacdnu = jacobian(dnu, states)

