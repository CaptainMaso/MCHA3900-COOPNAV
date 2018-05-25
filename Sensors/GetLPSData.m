function [y_lps,SR] = GetLPSData(in)
global map param

eta = in(1:6);
rBNn = eta(1:3);

y_lps   = zeros(map.LPS.N,1);
SR      = zeros(map.LPS.N);
for ii = 1:map.LPS.N
    rLBn          = map.LPS.locations(:,ii) - rBNn;
    y_lps(ii)     = norm(rLBn);
    SR(ii,ii)     = param.LPS.sigma;
end

