function [Y_mono, SR_mono] = mm_mono(Xm, Um)

auv_etanu   = Xm(1:12);
wamv_etanu  = Xm(16:27);
quad_etanu  = Xm(31:42);

auv_gyrobias    = Xm(13:15);
wamv_gyrobias   = Xm(28:30);
quad_gyrobias   = Xm(43:45);

auv_U   = Um(1:6);
wamv_U  = Um(7:12);
quad_U  = Um(13:18);

[Y_AUV, SR_AUV]     = mm_auv(auv_etanu, auv_U, auv_gyrobias, wamv_etanu(1:6));
[Y_WAMV, SR_WAMV]   = mm_wamv(wamv_etanu, wamv_U, wamv_gyrobias);
[Y_QUAD, SR_QUAD]   = mm_quad(quad_etanu, quad_U, quad_gyrobias, wamv_etanu(1:6));

%% Stack and Return
Y_mono = [Y_AUV;Y_WAMV;Y_QUAD];
SR_mono = blkdiag(SR_AUV, SR_WAMV, SR_QUAD);