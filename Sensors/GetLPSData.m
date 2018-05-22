function y_lps = GetLPSData(in)
global map

eta = in(1:6);
rBNn = eta(1:3);

y_lps = zeros(1,map.LPS.N);

for ii = 1:map.LPS.N
    rLBn          = map.LPS.locations(ii,:) - rBNn;
    y_lps(ii)     = norm(rLBn);
end