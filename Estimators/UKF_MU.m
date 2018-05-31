function [mu_post, S_post] = UKF_MU(Y, mu_pri, S_pri, U, measurementModel)

% Measurement Update

jointFunc               = @(x) augmentIdentityAdapter(measurementModel, x, U);    
[muxy,Syx]              = unscentedTransform(mu_pri, S_pri, jointFunc);    

[mu_post, S_post]       = conditionGaussianOnMarginal(muxy, Syx, Y);
