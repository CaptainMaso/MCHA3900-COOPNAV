function y_lps = GetLPSData(in)
global map

eta = in(1:6);
rBNn = eta(1:3);

y_lps = zeros(map.LPS.N,1);

for ii = 1:map.LPS.N
    rLBn          = map.LPS.locations(:,ii) - rBNn;
    y_lps(ii)     = norm(rLBn);
end