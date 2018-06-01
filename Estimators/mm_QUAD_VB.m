function [Y_VB, SR_VB] = mm_QUAD_VB(quad_etanu, ~, ~, wamv_eta)
global param
% function [Y_quad, SR_quad] = mm_quad(quad_etanu, quad_U, wamv_eta, quad_gyrobias)
Rnq     = eulerRotation(quad_etanu(4:6));
rWQn    = wamv_eta(1:3) - quad_etanu(1:3);
rWQq    = Rnq'*rWQn;

% Gets VB Data
[Y_VB,SR_VB]    = GetVBData(quad_etanu);

 if (norm(rWQq) == 0)
    vb_wamv = [0;0;0];
 else
     vb_wamv = rWQq/norm(rWQq);
 end
 
SR_VB   = blkdiag(SR_VB, param.VB.sigma);
Y_VB    = [Y_VB;vb_wamv];           % Adds normalised bearing vector from quad to wamv