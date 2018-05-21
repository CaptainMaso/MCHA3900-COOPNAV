function Xm_next = processModelMonolithic(Xm, Um)

X_AUV   = Xm(1:12); % Assume full 6DOF, unused states should have restoring forces on them
U_AUV   = Um(1:6);  % Assume fully actuated, underactuated systems should pad desired forces
                    % with zeros
X_WAMV  = Xm(13:24);
U_WAMV  = Um(7:12);

X_QUAD  = Xm(25:36);
U_QUAD  = Um(13:18);

X_bias  = Xm(37:end);   % Biases should be sorted later (But they're just a random walk for
                        % this section anyway

% AUV won't work atm since it's in the wrong format
dX_AUV  = zeros(12,1);%auv_detanu([X_AUV; 

dX_WAMV = wamv_detanu([X_WAMV; U_WAMV]);

dX_QUAD = quad_detanu([X_QUAD; U_QUAD]);

% Shitty Euler Approximation, but it's the best we've got for something
% this non-linear
dXm     = [dX_AUV;dX_WAMV;dX_QUAD;zeros(size(X_bias,1),1)];

Xm_next = eye(36 + size(X_bias,1)) + (1/param.sensor_sample_rate)*dXm;