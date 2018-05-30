function [mu_post, S_post] = UKF_MU(Y, mu_pri, S_pri, U, measurementModel,Mono_Sub_switch)

% Measurement Update
if Mono_Sub_switch == 0
    jointFunc               = @(x) augmentIdentityAdapter(measurementModel, x, U);
    [muxy,Syx]              = unscentedTransform(mu_pri(1:15), S_pri, jointFunc);
elseif Mono_Sub_switch == 1
    jointFunc               = @(x) augmentIdentityAdapter(measurementModel, x, U);    
    [muxy,Syx]              = unscentedTransform(mu_pri, S_pri, jointFunc);    
else
    error('ERROR, shits not right');
end
[mu_post, S_post]       = conditionGaussianOnMarginal(muxy, Syx, Y);
