function B = actuatorConfiguration
Ixi = 2;
Iyi = 1;
a1 = deg2rad(-10);
a2 = deg2rad(+10);
% Refer to the Thrust Configuration slide from the Week 2 Vehicle control slides
B = [cos(a1), cos(a2);
    sin(a1), sin(a2);
    -Iyi*cos(a1)-Ixi*sin(a1), Iyi*cos(a2)-Ixi*sin(a2)];

%Bdag = B'*inv(B*B')';

