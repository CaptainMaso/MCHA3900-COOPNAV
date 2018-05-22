function [mux,Sxx] = conditionGaussianOnMarginal(muyx,Syx,y)

% Cache structure used by linsolve
persistent s_ut_transa
if isempty(s_ut_transa)
    s_ut_transa = struct('UT',true,'TRANSA',true);
end

ny  = length(y);

S1  = Syx(1:ny,1:ny);
S2  = Syx(1:ny,ny+1:end);
Sxx = Syx(ny+1:end,ny+1:end);
muy = muyx(1:ny,:);
mux = muyx(ny+1:end,:);

%
% Corrected state mean
%
mux = mux + S2.'* ...
    linsolve(S1, y - muy, s_ut_transa); % Upper triangular solver for S1.'\(y - yhat)

% Note: K = Pxy/Pyy = (S2.'*S1)/(S1.'*S1) = S2.'/S1.'
% It is more efficient to evaluate
% S2.'*(S1.'\(y - yhat)) rather than (S2.'/S1.')*(y - yhat)