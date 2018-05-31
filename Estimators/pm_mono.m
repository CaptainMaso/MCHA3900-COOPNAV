function [Xm_next,Xm_SQ] = pm_mono(Xm, Um)
global param
auv_etanu   = Xm(1:12);
wamv_etanu  = Xm(16:27);
quad_etanu  = Xm(31:42);

% auv_gyrobias    = mono_X(13:15);
% wamv_gyrobias   = mono_X(28:30);
% quad_gyrobias   = mono_X(43:45);

auv_U   = Um(1:6);
wamv_U  = Um(7:12);
quad_U  = Um(13:18);

dX_AUV  = auv_detanu( [auv_U ; auv_etanu ]); 

dX_WAMV = wamv_detanu([wamv_U; wamv_etanu]);

dX_QUAD = quad_detanu([quad_U; quad_etanu]);

% Shitty Euler Approximation, but it's the best we've got for something
% this non-linear
dXm     = [dX_AUV;zeros(3,1);dX_WAMV;zeros(3,1);dX_QUAD;zeros(3,1)];

Xm_next = Xm + dXm./param.sensor_sample_rate;

Xm_SQ   = blkdiag(param.AUV.SQeta,  param.AUV.SQnu,  param.IMU.SQbias, ...
                  param.WAMV.SQeta, param.WAMV.SQnu, param.IMU.SQbias, ...
                  param.QUAD.SQeta, param.QUAD.SQnu, param.IMU.SQbias);