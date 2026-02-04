simulationTime = 1000;
dt = 0.1;
Ncase = 3800;

%target motion params
initialAngularVero = zeros([Ncase 3]);
tmp = linspace(0, 3.7, 38);
for i=1:Ncase
    initialAngularVero(i,2) = tmp(ceil(i/100));
end
initialAngularVero(:,3) = linspace(0.2, 0.2, Ncase);

initialChaserPose = repmat([-30 0 0]',1,Ncase);
initialChaserVelo = repmat([0 0 0]',1,Ncase);

%params for potential field
Ka = 1;
Kr = 1;
Krotation = 1;
attractiveShapingMatrix_0 = [0.01 0 0; 0 0.01 0; 0 0 0.01];
keepOutHightMatrix = 10*[0.0125 0 0; 0 0.0125 0; 0 0 0.0125];
keepOutWidthMatrix = 0.25*[20 0 0; 0 20 0; 0 0 20];
rhoUpper = [1 1 1 1 1 1];
rhoUnder = -rhoUpper;
rhoUpper_r = [0.05 0.05 0.05 0.05 0.05 0.05];
rhoUnder_r = -rhoUpper_r;
R_matrix_r_0 = [0.01 0 0;
                0 0.01 0;
                0 0 0.01];

vLim = 0.1;
uLim = 0.05;
Ku = 5;
% uUpper = [0.005 0.005 0.005]; %control upper limit (m/s^2)
% uLower = [-0.005 -0.005 -0.005]; %control lower limit (m/s^2)

enableNavigationError_position = true;
enableNavigationError_velocity = true;
enableNavigationError_attitude = true;
enableNavigationError_motion = true;