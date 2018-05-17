function y_lps = GetLPSData(in)
global map param

eta = in(1:6);

x = eta(1:3);

y_lps = zeros(1,map.LPS.N);

for ii = 1:map.LPS.N
   y_lps(ii) = norm(map.LPS.locations(ii,:) - x) + param.LPS.sigma*rand;
end

