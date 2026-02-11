% Maximum simulation time
simulationTime = 1000;
% Step time
dt = 0.1;
% Number of simulation cases
Ncase = 7;

% target initial angular velocity
% Define in target body frame
initialAngularVelo = zeros([Ncase 3]);
initialAngularVelo(:,2) = 0;

%chaser state (Hill's frame)
initialChaserPose = repmat([-30 0 0]',1,Ncase);
initialChaserVelo = repmat([0 0 0]',1,Ncase);

initialChaserPose(1,:) = -30*cos(pi/6*(0:1:6));
initialChaserPose(2,:) = 30*sin(pi/6*(0:1:6));

% params for guidance
%   Gain
Ka = 1;
Kr = 1;
Krotation = 1;
%   Adaptive Attractive Potential Field
attractiveShapingMatrix_0 = [0.1 0 0; 0 0.1 0; 0 0 0.1]/30;
rhoUpper = [1 1 1 1 1 1];
rhoUnder = -rhoUpper;
%   Repulsive Potential Fiel
keepOutHightMatrix = 0.1*[0.1 0 0; 0 0.1 0; 0 0 0.1];
keepOutWidthMatrix = 20*[1 0 0; 0 1 0; 0 0 1];
%   Rotational Vector Field
rhoUpper_r = [1 1 1 1 1 1]*0.03;
rhoUnder_r = -rhoUpper_r;
R_matrix_r_0 = [0.01 0 0;
                0 0.01 0;
                0 0 0.01];
% Approach reference velocity [m/s]
vLim = 0.1;
% control input limit [m/s^2]
uLim = 0.05;
% control gain
Ku = 5;

% Navigation errors
% Detail of navigation errors are defined in ApproachSim.m
enableNavigationError_position = false;
enableNavigationError_velocity = false;
enableNavigationError_attitude = false;
enableNavigationError_motion = false;