function [Xm_next,Xm_SQ] = pm_mono_oosm(Xm, Um)
global param
auv_etanu_t0   = Xm(1:12);
wamv_etanu_t0  = Xm(16:27);
quad_etanu_t0  = Xm(31:42);

% auv_gyrobias    = mono_X(13:15);
% wamv_gyrobias   = mono_X(28:30);
% quad_gyrobias   = mono_X(43:45);

auv_U   = Um(1:6);
wamv_U  = Um(7:12);
quad_U  = Um(13:18);

dX_AUV  = auv_detanu( [auv_U ; auv_etanu_t0 ]); 

dX_WAMV = wamv_detanu([wamv_U; wamv_etanu_t0]);

dX_QUAD = quad_detanu([quad_U; quad_etanu_t0]);

% Shitty Euler Approximation, but it's the best we've got for something
% this non-linear
dXm0     = [dX_AUV;zeros(3,1);dX_WAMV;zeros(3,1);dX_QUAD;zeros(3,1)];

Xm_next0 = Xm(1:45) + dXm0./param.sensor_sample_rate;

Xm_SQ0   = blkdiag(param.AUV.SQeta,  param.AUV.SQnu,  param.IMU.SQbias, ...
                  param.WAMV.SQeta, param.WAMV.SQnu, param.IMU.SQbias, ...
                  param.QUAD.SQeta, param.QUAD.SQnu, param.IMU.SQbias);
              
% OOSM Section
Xm_next = nan(45*param.maxDelay,1);
Xm_SQ   = zeros(45*param.maxDelay,45*param.maxDelay);

Xm_next(1:45) = Xm_next0;
Xm_SQ(1:45,1:45) = Xm_SQ0;
for k_n = 0:param.maxDelay-2
    Xm_next((1:45) + (k_n+1)*45) = Xm((1:45) + k_n*45);
end
              
              