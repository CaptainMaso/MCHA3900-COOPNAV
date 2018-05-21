function [ array_time ] = Hydroacoustics( eta, nu, dnu )
% Only really uses AUV to USV relative eta, adds noise based on
% radial distance, returns hydroacoustic data from each hydrophone in array
% assume AUV has a Sonotronics EMT-01-03 marker transmitter
% persistent prev_eta;
% 
% ts = 0.01;
v = 1500; % speed of sound underwater m/s
d = 9.375e-3;  %  minimum distance/2 between hydrophone array according to nyquist theorem for 40khz signal
% toffset = sqrt(eta(1)^2 + eta(2)^2 + eta(3)^2)/v;
array = [1 1 1; 1 -1 1; -1 1 1; -1 -1 1; 1 1 -1; 1 -1 -1; -1 1 -1; -1 -1 -1].*d;
noise = randn(8,3)./SignaltoNoiseRatio(eta);
array_time = zeros(8,1);
for ii = 1:8
    eta2array_dist = sqrt((eta(1)-array(ii,1) - noise(ii,1))^2 + (eta(2)-array(ii,2) - noise(ii,1))^2 + (eta(3)-array(ii,3) - noise(ii,3))^2);
    array_time(ii) = eta2array_dist/v;
end
end

