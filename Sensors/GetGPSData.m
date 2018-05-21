function y_gps = GetGPSData(in)
global param
%in(1:6) -> eta
%in(7:12) -> nu
%in(13:18) -> dnu

eta = in(1:6);
nu = in(7:12);
dnu = in(13:18);

if (eta(1) > 500); eta(1)=500; elseif (eta(1) < -500); eta(1)=-500; end
if (eta(2) > 500); eta(2)=500; elseif (eta(2) < -500); eta(2)=-500; end
if (eta(1) > 100); eta(3)=100; elseif (eta(3) < -100); eta(3)=-100; end

y_gps = [eta(1);eta(2);eta(3)];