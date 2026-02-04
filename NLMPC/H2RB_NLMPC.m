close all
clear all

Ncase = 1;

%simulation time vec
simulationTime = 1000;
dt = 0.1;
steps = simulationTime / dt + 1;
time = linspace(0,simulationTime,steps);

%define the initial params
utility = utility();
target = target_KOZ1();
h2rb_initialState = target.initialState();
targetVelocityNorm = norm(h2rb_initialState(1:3));
semiMajorAxis = norm(h2rb_initialState(4:6));
initialChaserPose = [-30 0 0];
initialChaserVelo = [0 0 0];
omega = targetVelocityNorm/semiMajorAxis;
target.omega = omega;

%time deriver of the target motion.
[~, x] = ode45(@(t,x)target.EOM(t,x),time,h2rb_initialState);
targetPosition = x(:,4:6);
targetVelocity = x(:,1:3);
clear x

%%
%time deriver of the target rotating motion.
angularVeloRandom = zeros([Ncase 3]);
x0 = zeros([Ncase 10]);

x0_ = [0 -1*omega 0 ... target angular velocity
    1/sqrt(2) 0 1/sqrt(2) 0 ... target attitude quaternion that represent point rotation
    0 0 0]; ... torque input
for i=1:Ncase
    % angularVeloRandom(i,:) = random('uniform',[-0.5 -0.5], [0.5 0.5]);
    x0(i,:) = x0_;
    x0(i,1:3) = x0(i,1:3) + angularVeloRandom(i,:)*pi/180;
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
tic
parfor j=1:Ncase
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

%%
attractivePose = [0 0 6];
%%
clear nlobj
j=1;

chaser_x0 = [initialChaserPose [0 0 0]];
nlobj = nlmpc(length(chaser_x0(1,:)),6,3);
nlobj.Model.StateFcn = @(x,u)EOM(0,x,u,omega);
nlobj.Model.OutputFcn = @(x,u) x;
nlobj.Ts = dt;
r=zeros([steps,6]);
attractivePoseMat = repmat(attractivePose,steps,1);
r(:,1:3) = rotateframe(quaternion_BODY2RTN(:,j),attractivePoseMat);

x = chaser_x0;
mv_ = zeros([3 1]);
validateFcns(nlobj,chaser_x0,mv_)
mv = zeros([steps 3]);
path = zeros([steps 3]);
%%
for i=1:steps-1
    mv_=nlmpcmove(nlobj,x,mv_,r(i,:));
    mv(i,:) = mv_;

    [~,xhist] = ode45(@(t,x)EOM(t,x,mv_,omega),[0 dt],x);
    x = xhist(end,:);
    path(i,:)=x(1:3);
end

%%
figure(Name="mv")
plot(time,mv(:,1));
hold on
plot(time,mv(:,1));
plot(time,mv(:,1));
legend;

figure(Name="path")
plot(time,path(:,1));
hold on
plot(time,path(:,2));
plot(time,path(:,3));
legend;