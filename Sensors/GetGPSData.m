function [y_gps,SR] = GetGPSData(in)
global param
%in(1:6) -> eta
%in(7:12) -> nu
%in(13:18) -> dnu

eta = in(1:6);

y_gps = [eta(1);eta(2);eta(3)];

SR = param.GPS.sigma;