function y_VB = GetVBData(in)
global map

etaBNn = in(1:6);
rBNn = etaBNn(1:3);
thetaNB = etaBNn(4:6);
Rnb = eulerRotation(thetaNB);

y_VB = zeros(3,map.VB.N);

for ii = 1:map.VB.N
    rVBn = map.VB.locations(:,ii) - rBNn;
    rVBb = Rnb'*rVBn;
    y_VB(:,ii) = rVBb/norm(rVBb);  % Stack vector outputs
end

y_VB = y_VB(:);