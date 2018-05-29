function [etanu_next, SQ] = pm_quad(quad_etanu, quad_u)
global param

detanu_QUAD = quad_detanu([quad_u; quad_etanu]);
etanu_next = quad_etanu + detanu_QUAD/param.sensor_sample_rate;

SQ = blkdiag(param.QUAD.SQeta, param.QUAD.SQnu);