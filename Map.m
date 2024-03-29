%% Initialise map and load bits
% Max and min and other elements
map.max = [100,   100,  25]';          % (N,E,D) (m)
map.min = [-100, -100, -25]';          % (N,E,D) (m)

% First bits to load are the LPS tags
map.LPS.N = 3;
map.LPS.locations = [(rand(map.LPS.N,2))*(diag(map.max(1:2)-map.min(1:2))) + ones(map.LPS.N,2)*diag(map.min(1:2)), zeros(map.LPS.N,1)]';

% After that are the buoy locations (I.E. Visual Bearing Objects)
map.VB.N = 3;
map.VB.locations = [(rand(map.VB.N,2))*(diag(map.max(1:2)-map.min(1:2))) + ones(map.VB.N,2)*diag(map.min(1:2)), zeros(map.VB.N,1)]';
