tic
%simulation time vec
steps = simulationTime / dt + 1;
time = linspace(0,simulationTime,steps);

%define the initial params
utility = utility();
target = target_KOZ1();
h2rb_initialState = target.initialState();
targetVelocityNorm = norm(h2rb_initialState(1:3));
semiMajorAxis = norm(h2rb_initialState(4:6));
omega = targetVelocityNorm/semiMajorAxis;
target.omega = omega;

%time deriver of the target motion.
[~, x] = ode45(@(t,x)target.EOM(t,x),time,h2rb_initialState);
targetPosition = x(:,4:6);
targetVelocity = x(:,1:3);
clear x

%%
%time deriver of the target rotating motion.
x0 = zeros([Ncase 10]);

% -omega is used to fix the target in hill frame.
x0_ = [0 -1*omega 0 ... target angular velocity in body frame
    1/sqrt(2) 0 1/sqrt(2) 0 ... target attitude quaternion that represent point rotation
    0 0 0]; ... torque input
 
for i=1:Ncase
    x0(i,:) = x0_;
    x0(i,1:3) = x0(i,1:3) + initialAngularVelo(i,:)*pi/180;
end

targetAngularVelocity = zeros([length(time) 3 Ncase]);
targetAttitude = zeros([length(time) 4 Ncase]);
lamda = @(t,x)target.EOM_rotation(t,x,targetPosition, time);
parfor i=1:Ncase
    [~, res] = ode45(@(t,x)lamda(t,x),time,x0(i,:).');
    targetAngularVelocity(:,:,i) = res(:,1:3);
    targetAttitude(:,:,i) = res(:,4:7);
end
clear lamda;

%%
%Calculate the frame transfer quaternions.
%These quaternions represent frame rotations.
quaternion_ECI2RTN = zeros([steps 1],"quaternion");
for i=1:length(time)
    quaternion_ECI2RTN(i) = conj(quaternion(utility.getRotationMatrixRTN2IJK(targetPosition(i,:).',targetVelocity(i,:).'),"rotmat","frame"));
end

%%
quaternion_ECI2BODY = zeros([steps Ncase],"quaternion");
quaternion_BODY2RTN = zeros([steps Ncase],"quaternion");

quaternion_ECI2BODY = (quaternion(targetAttitude(:,1,:),targetAttitude(:,2,:),targetAttitude(:,3,:),targetAttitude(:,4,:)));
quaternion_ECI2BODY = reshape(quaternion_ECI2BODY,[steps Ncase]);
%%
parfor j=1:Ncase
    % quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);
    quaternion_ECI2BODY_ = quaternion_ECI2BODY(:,j);
    quaternion_ECI2RTN_ = quaternion_ECI2RTN;

    quaternion_tmp = zeros([steps 2],"quaternion");
    quaternion_tmp(:,1) = conj(quaternion_ECI2BODY_);
    quaternion_tmp(:,2) = (quaternion_ECI2RTN_);
    quaternion_BODY2RTN_ = prod(quaternion_tmp,2);

    quaternion_BODY2RTN_ = reshape(quaternion_BODY2RTN_,[steps 1]);
    quaternion_BODY2RTN(:,j) = quaternion_BODY2RTN_;
end
clear quaternion_ECI2RTN_
clear quaternion_ECI2BODY_


%control parameter
attractivePose = [0 0 6.5].';
R_matrix_0 = sqrt(attractiveShapingMatrix_0);
rho = zeros([6 steps, Ncase]);
% R = R_matrix_0;
% R_dot = zeros(size(R));

rho_r = zeros([6 steps, Ncase]);
% R_r = R_matrix_r_0;
% R_dot = zeros(size(R_r));

%defines params matrix
path = zeros([3 steps Ncase]);
velo = zeros([3 steps Ncase]);
u = zeros([3 steps-1, Ncase]);
isArrived = zeros([1,Ncase]);
indexStamp = zeros([3,Ncase]);

clear targetAttitude
toc