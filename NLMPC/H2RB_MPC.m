close all
clear all

tic
%simulation time vec
simulationTime = 1000;
dt = 0.01;
steps = simulationTime / dt + 1;
time = linspace(0,simulationTime,steps);

%define the initial params
utility = utility();
target = target();
h2rb_initialState = target.initialState();
targetVelocityNorm = norm(h2rb_initialState(1:3));
semiMajorAxis = norm(h2rb_initialState(4:6));
initialROE = [0 -0/semiMajorAxis 30 0 30 0];
[initialChaserPose, initialChaserVelo] = utility.translateROE2PoseVeloInRTN(initialROE,0,semiMajorAxis,targetVelocityNorm);
omega = targetVelocityNorm/semiMajorAxis;

%time deriver of the target motion.
[~, x] = ode45(@(t,x)target.EOM(t,x),time,h2rb_initialState);
target.position = x(:,4:6);
target.velocity = x(:,1:3);
target.t = time;

%%
%time deriver of the target rotating motion.
Ncase = 1;
angularVeloRandom = zeros([Ncase 2]);
x0 = zeros([Ncase 10]);

x0_ = [0 -1*omega 0 ... target angular velocity
    1/sqrt(2) 0 1/sqrt(2) 0 ... target attitude quaternion that represent point rotation
    0 0 0]; ... torque input
for i=1:Ncase
    % angularVeloRandom(i,:) = random('uniform',[-0.5 -0.5], [0.5 0.5]);
    x0(i,:) = x0_;
    x0(i,1:2) = x0(i,1:2) + angularVeloRandom(i,:)*pi/180;
end

targetAngularVelocity = zeros([length(time) 3 Ncase]);
targetAttitude = zeros([length(time) 4 Ncase]);
lamda = @(t,x)target.EOM_rotation(t,x,target.position, target.t);
parfor i=1:Ncase
    [~, res] = ode45(@(t,x)lamda(t,x),time,x0(i,:).');
    targetAngularVelocity(:,:,i) = res(:,1:3);
    targetAttitude(:,:,i) = res(:,4:7);
end
target.angularVelocity = targetAngularVelocity;

%%
%Calculate the frame transfer quaternions.
%These quaternions represent frame rotations.
quaternion_ECI2RTN = zeros([steps 1],"quaternion");
for i=1:length(time)
    quaternion_ECI2RTN(i) = conj(quaternion(utility.getRotationMatrixRTN2IJK(target.position(i,:).',target.velocity(i,:).'),"rotmat","frame"));
end

%%
quaternion_ECI2BODY = zeros([steps Ncase],"quaternion");
quaternion_BODY2RTN = zeros([steps Ncase],"quaternion");

quaternion_ECI2BODY = (quaternion(targetAttitude(:,1,:),targetAttitude(:,2,:),targetAttitude(:,3,:),targetAttitude(:,4,:)));
quaternion_ECI2BODY = reshape(quaternion_ECI2BODY,[steps Ncase]);
%%
tic
for j=1:Ncase
    % quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);
    quaternion_ECI2BODY_ = quaternion_ECI2BODY(:,j);
    quaternion_ECI2RTN_ = quaternion_ECI2RTN;

    quaternion_tmp = zeros([steps 2],"quaternion");
    quaternion_tmp(:,1) = conj(quaternion_ECI2BODY_);
    quaternion_tmp(:,2) = (quaternion_ECI2RTN_);
    quaternion_BODY2RTN_ = prod(quaternion_tmp,2);
    % for i=1:length(quaternion_BODY2RTN_)
    %     quaternion_BODY2RTN_(i) = conj(quaternion_ECI2BODY_(i))*(quaternion_ECI2RTN_(i));
    % end
    quaternion_BODY2RTN_ = reshape(quaternion_BODY2RTN_,[steps 1]);
    quaternion_BODY2RTN(:,j) = quaternion_BODY2RTN_;
end
toc
%%
target.quaternion_ECI2BODY = quaternion_ECI2BODY;
target.quaternion_BODY2RTN = quaternion_BODY2RTN;
target.quaternion_ECI2RTN = quaternion_ECI2RTN;

clear targetAttitude
clear quaternion_ECI2BODY
clear quaternion_BODY2RTN
clear quaternion_ECI2RTN

%%
attractivePose = [0 0 6];
%%
clear nlobj
% chaser_x0 = [initialChaserPose.' [0 0 0].'];

A = [0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 0 0];
B = [0 0 0;
    0 0 0;
    0 0 0;
    1 0 0;
    0 1 0;
    0 0 1];
C = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];
D = [0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0;
    0 0 0];
r=zeros([steps,6]);
attractivePoseMat = repmat(attractivePose,steps,1);
r(:,1:3) = rotateframe(target.quaternion_BODY2RTN(:,j),attractivePoseMat);

mpcController = mpc(ss(A,B,C,D), 0.01);
mpcController.Weights.OutputVariables = [1 1 1 3 3 3];
mpcController.PredictionHorizon = 100;
mpcController.ControlHorizon = 5;
mpcController.Model.Nominal.X = [-30 0 0 0 0 0];
mpcController.Model.Nominal.Y = [-30 0 0 0 0 0];
mpcController.ManipulatedVariables(1).Min = -0.05;
mpcController.ManipulatedVariables(1).Max = 0.05;
mpcController.ManipulatedVariables(2).Min = -0.05;
mpcController.ManipulatedVariables(2).Max = 0.05;
mpcController.ManipulatedVariables(3).Min = -0.05;
mpcController.ManipulatedVariables(3).Max = 0.05;

% mpcController.Model.Plant = minreal(mpcController.Model.Plant);
sim(mpcController,steps,r);
