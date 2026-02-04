simulationTime = 1000;
dt = 0.1;
Ncase = 1;

%target motion params
initialAngularVero = zeros([Ncase 3]);
initialAngularVero(1,:) = [0 2.6 0];

%chaser state
initialChaserPose = repmat([-30 0 0]',1,Ncase);
initialChaserVelo = repmat([0 0 0]',1,Ncase);

%params for potential field
Ka = 1;
Kr = 1;
Krotation = 1;
attractiveShapingMatrix_0 = [0.1 0 0; 0 0.1 0; 0 0 0.1]/30;
keepOutHightMatrix = 0.1*[0.1 0 0; 0 0.1 0; 0 0 0.1];
keepOutWidthMatrix = 20*[1 0 0; 0 1 0; 0 0 1];
rhoUpper = [1 1 1 1 1 1];
rhoUnder = -rhoUpper;
rhoUpper_r = [1 1 1 1 1 1]*0.03;
rhoUnder_r = -rhoUpper_r;
R_matrix_r_0 = [0.01 0 0;
                0 0.01 0;
                0 0 0.01];

vLim = 0.1;
uLim = 0.05;
Ku = 5;
% uUpper = [0.005 0.005 0.005]; %control upper limit (m/s^2)
% uLower = [-0.005 -0.005 -0.005]; %control lower limit (m/s^2)

enableNavigationError_position = false;
enableNavigationError_velocity = false;
enableNavigationError_attitude = false;
enableNavigationError_motion = false;