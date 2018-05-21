function [muy,Syy] = affineTransform(mux,Sxx,h)

%
% Square Root Affine Transform of y = h(x) + v
% where v ~ N(0,R) = N(0,SR.'*SR)
% given Jacobian C = dh/dx at x = mux
%

[muy,C,SR] = h(mux);
ny = length(muy);

% R = chol(A'A) = qr(A)     Square-root Kalman filtering: slide 6
R = triu(qr([Sxx*C.';SR],0));           % Square-root Kalman filtering: slide 9
Syy = R(1:ny,:);                        % upper Cholesky factor of Pyy

Syy = sign(diag(Syy)).*Syy; % Choose factor with positive diagonal (QR is not unique)