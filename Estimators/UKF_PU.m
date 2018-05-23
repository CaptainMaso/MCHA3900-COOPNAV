function [mu_next, S_next] = UKF_PU(mu_pri, S_pri, U, processModel)

% Prediction Update
processFunc  = @(x) processModel(mu_pri, U);
[mu_next, S_next] = unscentedTransform(mu_pri, S_pri, processFunc);