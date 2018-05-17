function y_VB = GetVBData(in)
global map param

eta = in(1:6);
x = eta(1:3)
Theta = eta(4:6);
RBNb = eulerRotation(Theta);

y_VB = zeros(3,map.VB.N);

for ii = 1:map.VB.N
    vec = RBNb'*(map.VB.locations(ii,:)' - x) + param.VB.sigma*[randn;randn;randn];
    y_VB(:,ii) = vec/norm(vec);
end