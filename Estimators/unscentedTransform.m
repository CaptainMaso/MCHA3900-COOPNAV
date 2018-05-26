function [muy,Syy] = unscentedTransform(mux,Sxx,h,c)

%
% Square Root Unscented Transform of y = h(x) + v
% where v ~ N(0,R) = N(0,SR.'*SR)
% using 2*n + 1 sigma points and output constraints y <-- c(y)
%

n = length(mux);          	% Length of input vector
nsigma = 2*n+1;             % Number of sigma points

% Unscented transform parameters
alpha = 1;
kappa = 0;
lambda = alpha^2*(n + kappa) - n;
gamma = sqrt(n + lambda);
beta = 2;

% Mean weights
Wm = [repmat(1/(2*(n+lambda)),1,2*n), lambda/(n+lambda)];

% Covariance weights
Wc = [repmat(1/(2*(n+lambda)),1,2*n), lambda/(n+lambda) + (1-alpha^2+beta)];

% Generate sigma points
xsigma = zeros(n,nsigma);   % Input sigma points
for i = 1:n
    xsigma(:,i)   = mux + gamma*Sxx(i,:).';
    xsigma(:,i+n) = mux - gamma*Sxx(i,:).';
end
xsigma(:,2*n+1) = mux;

% Apply constraints
if nargin >= 4
    for i = 1:nsigma
        xsigma(:,i) = c(xsigma(:,i));
    end
end

% Transform the sigma points through the function
[temp,SR] = h(xsigma(:,nsigma));  	% Use function eval at mean to extract SR and
ny = length(temp);                      % determine output dimension and
ysigma = zeros(ny,nsigma);              % initialise output sigma points
ysigma(:,nsigma) = temp;
for i = 1:nsigma-1
    [ysigma(:,i),~] = h(xsigma(:,i));
end

% Unscented mean
muy = sum(Wm.*ysigma,2);

% Compute conditional mean and sqrt covariance
dysigma = [realsqrt(Wc).*(ysigma - muy), SR];
R = triu(qr(dysigma.',0));
Syy = R(1:ny,:);

Syy = sign(diag(Syy)).*Syy; % Choose factor with positive diagonal (QR is not unique)