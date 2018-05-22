function Ym = measurementModelMonolithic(Xm, Um)
                    % Assume full 6DOF, unused states should have restoring forces on them
U_AUV   = Um(1:6);  % Assume fully actuated, underactuated systems should pad desired forces
                    % with zeros
U_WAMV  = Um(7:12);

U_QUAD  = Um(13:18);

Y_AUV   = measurementModelAUV(Xm, U_AUV);
Y_WAMV  = measurementModelWAMV(Xm, U_WAMV);
Y_QUAD  = measurementModelQUAD(Xm, U_QUAD);

Ym = [Y_AUV;Y_WAMV;Y_QUAD];