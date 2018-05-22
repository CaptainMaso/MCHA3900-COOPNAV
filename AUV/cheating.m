function [ y ] = cheating( in )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
eta_nu = in(1:10);
dnu = in(11:15);
fake = in(16:18);

y = [eta_nu(1:4); fake(1); eta_nu(5); eta_nu(6:9); fake(2); eta_nu(10); dnu(1:4); fake(3); dnu(5)];

end

