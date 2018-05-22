function [yx,CI,SRR] = augmentIdentityAdapter(f,x,u,param)

[y,C,SR] = f(x,u,param);

yx = [y;x];

nx = length(x);
CI = [C;eye(nx)];

ny = length(y);
SRR = zeros(nx+ny);
SRR(1:ny,1:ny) = SR;