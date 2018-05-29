function [Y_mono, SR_mono] = mm_mono(mono_X, Um)

auv_etanu   = mono_X(1:12);
wamv_etanu  = mono_X(13:24);
quad_etanu  = mono_X(25:36);

auv_gyrobias    = mono_X(37:39);
wamv_gyrobias   = mono_X(40:42);
quad_gyrobias   = mono_X(43:45);

auv_U   = Um(1:6);
wamv_U  = Um(7:12);
quad_U  = Um(13:18);

[Y_AUV, SR_AUV]     = mm_auv(auv_etanu,   auv_U,  wamv_etanu(1:6), auv_gyrobias);
[Y_WAMV, SR_WAMV]   = mm_wamv(wamv_etanu, wamv_U, wamv_gyrobias);
[Y_QUAD, SR_QUAD]   = mm_quad(quad_etanu, quad_U, wamv_etanu(1:6), quad_gyrobias);

%% Stack and Return
Y_mono = [Y_AUV;Y_WAMV;Y_QUAD];
SR_mono = blkdiag(SR_AUV, SR_WAMV, SR_QUAD);