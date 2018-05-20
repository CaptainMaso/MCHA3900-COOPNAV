%% Main function to generate tests
function tests = thrusterTest
tests = functiontests(localfunctions);
end

%% Test Functions
function testConflictedConfiguration(testCase)
u = [1;1;1];   	% Thruster forces [N]
actual = actuatorModel(u);
expected = [0;0];
verifyEqual(testCase, actual, expected, 'AbsTol', 10*eps, ...
    'Expected no body forces for conflicted actuator configuration');
end

function testSurgeConfiguration(testCase)
u = [-1;-1;1];	% Thruster forces [N]
actual = actuatorModel(u);
expected = [4/sqrt(2);0];
verifyEqual(testCase, actual, expected, 'AbsTol', 10*eps, ...
    'Expected pure surge thrust for forward actuator configuration');
end

function testSwayConfiguration(testCase)
u = [-1;1;1];    % Thruster forces [N]
actual = actuatorModel(u);
expected = [0;4/sqrt(2)];
verifyEqual(testCase, actual, expected, 'AbsTol', 10*eps, ...
    'Expected pure sway thrust for starboard actuator configuration');
end

function testControlAllocation(testCase)
tau3 = [1;2;3];     % Body forces (surge force [N], sway force [N], yaw torque [Nm])
actual = actuatorModel(controlAllocation(tau3));
expected = tau3;
verifyEqual(testCase, actual, expected, 'AbsTol', 10*eps, ...
    'Expected control allocation to produce thruster forces that implement demanded body forces');
end

function testLyLxValues(testCase)
u = [0.5;0;0.5];     % Body forces (surge force [N], sway force [N], yaw torque [Nm])
actual = actuatorModel(u);
expected = [0;0];
verifyEqual(testCase, actual, expected, 'AbsTol', 10*eps, ...
    'Expected control allocation to produce thruster forces that implement demanded body forces');
end

%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
end

function teardownOnce(testCase)  % do not change function name
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end