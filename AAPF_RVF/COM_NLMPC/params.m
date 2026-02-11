simulationTime = 1000;
dt = 0.1;
Ncase = 75*21;

% Initial condition
att_mat = zeros(3, 75, 21);
att_mat(2,:,:) = repmat(-3.7:0.1:3.7,1,1,21) * pi/180;
att_mat(3,:,:) = repmat(-1:0.1:1,75,1) * pi/180;
x_init_att(5:7,:) = reshape(att_mat,3,Ncase);

initialChaserPose = repmat([-30 0 0]',1,Ncase);
initialChaserVelo = repmat([0 0 0]',1,Ncase);

%target motion params
initialAngularVelo = zeros([Ncase 3]);
att_mat = zeros(3, 75, 21);
att_mat(2,:,:) = repmat(-3.7:0.1:3.7,1,1,21);
att_mat(3,:,:) = repmat(-0.1:0.01:0.1,75,1);
initialAngularVelo(:,:) = reshape(att_mat,3,Ncase)';

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

enableNavigationError_position = false;
enableNavigationError_velocity = false;
enableNavigationError_attitude = false;
enableNavigationError_motion = false;