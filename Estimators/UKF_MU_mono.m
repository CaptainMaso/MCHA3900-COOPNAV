function [mu_post, S_post] = UKF_MU_mono(Y, mu_pri, S_pri, U)

% Measurement Update
g = @(x,u)measurementModelMonolithic(x,u);
jointFunc               = @(x) augmentIdentityAdapter(g, x, U);
[muxy,Syx]              = unscentedTransform(mu_pri, S_pri, jointFunc);
[mu_post, S_post]       = conditionGaussianOnMarginal(muxy, Syx, Y);