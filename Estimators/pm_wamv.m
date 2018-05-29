function [etanu_next, SQ] = pm_wamv(wamv_etanu, wamv_u)
global param

detanu_WAMV = wamv_detanu([wamv_u; wamv_etanu]);
etanu_next = wamv_etanu + detanu_WAMV/param.sensor_sample_rate;

SQ = blkdiag(param.WAMV.SQeta, param.WAMV.SQnu);