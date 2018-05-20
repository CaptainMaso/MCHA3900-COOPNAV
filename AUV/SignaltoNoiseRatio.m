function [ SNR ] = SignaltoNoiseRatio( eta )
%UNTITLED3 Summary of this function goes here
% everything is in khz, km

T = 26.667; % ocean temp, assume a sunny day in Honolulu
D = 5e-3;   % operating depth (km), assume to be a constant 5 m
f = 40;     % freq of transmitter
f1 = 1.1;   % relaxation freq of boric acid in ocean
f2 = 75.6;  % relaxation freq of magnesium sulfate in ocean
etak = eta./1000;   

SL = 161.5;

avis = 4.9e-4*f^2*exp(-T/27+D/17);
ab = 0.106*f1*f^2/(f1^2+f^2);           
am = 0.52*(1+T/43)*f2*f^2/(f2^2+f^2)*exp(-D/6);
R = sqrt(etak(1)^2 + etak(2)^2 + etak(2)^2);
TLgeom = 20*log10(R);
TL = TLgeom + (avis + ab + am)*R;

NL = 75;    

DI = 25;

SNRdb = SL - TL - NL + DI;
SNR = sqrt(10^(SNRdb/10));      %take the 1/sqrt of signal to noise ratio as standard deviation of signal,https://www.sciencedirect.com/science/article/pii/S095741741500768X#bib0031, eq(11)

end

