function [yx,SRR] = augmentIdentityAdapter(f,x,u)
[y,SR] = f(x,u);

yx = [y;x];

nx = length(x);
ny = length(y);
SRR = zeros(nx+ny);
SRR(1:ny,1:ny) = SR;