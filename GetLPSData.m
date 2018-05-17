function y_lps = GetLPSData(in)
global map

eta = in(1:6);
nu  = in(7:12);
dnu = in(13:18);

LPS = map.LPS.locations;

x = eta(1:3);

y_lps = zeros(1,map.LPS.N);

for ii = 1:map.LPS.N
   y_lps(ii) = norm(LPS(ii,:) - x);
end

