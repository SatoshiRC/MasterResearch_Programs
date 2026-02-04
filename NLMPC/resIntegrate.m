directory = "log/sim4/";
% subFolders = ["0108000522", "0109215857"];
% windowNum = 1;
subFolders = ["0114122414", "0108012046"];
windowNum = 3;

Ncase = 75*21;
simulationTime =1000;
simulation_dt = 0.1;
steps = simulationTime / simulation_dt;
wind = 525;
activeRange = wind*(windowNum-1)+1:1:wind*windowNum;


x_init_att = zeros(7, Ncase);
x_init_att(1:4,:) = repmat([0.5; -0.5; -0.5; 0.5],1,Ncase);
att_mat = zeros(3, 75, 21);
att_mat(2,:,:) = repmat(-3.7:0.1:3.7,1,1,21) * pi/180;
att_mat(3,:,:) = repmat(-1:0.1:1,75,1) * pi/180;
x_init_att(5:7,:) = reshape(att_mat,3,Ncase);

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

att_fun = attitudeDynamics(simulation_dt, omega, target.InatiaMatrix());

nx = 6; %[p_R, p_T, p_N, v_R, v_T, v_N]
nu = 3; %[a_R, a_T, a_N]
np = 7; %[q_w, q_x, q_y, q_z, omega_X, omega_Y, omega_Z]

load("log/sim4/0114122414.mat");
state_log__ = zeros(size(state_log));
input_log__ = zeros(size(input_log));
constraint_log__ = zeros(size(constraint_log));
attitude_schedule_ = zeros(size(attitude_schedule));
index_end__ = zeros(size(index_end));

for i=1:length(subFolders)
    files = dir(directory+subFolders(i)+"/*.mat");
    
    for j=1:length(files)
        tmp = files(j).name;
        tmp = split(tmp, ".");
        tmp = split(tmp(1), "_");
        index = str2double(tmp(2));
        
        load(files(j).folder + "/" + files(j).name, "state_log_")
        state_log__(:,:,index) = state_log_;
        load(files(j).folder + "/" + files(j).name, "input_log_")
        input_log__(:,:,index) = input_log_;
        load(files(j).folder + "/" + files(j).name, "constraint_log_")
        constraint_log__(:,:,index) = constraint_log_;
        load(files(j).folder + "/" + files(j).name, "index_end_")
        index_end__(index) = index_end_;
    end
end

attitude_schedule = zeros(np, steps + predictHolizon*mpc_dt/simulation_dt + 1, Ncase);
for i=activeRange    
    % Pre-compute attitude and angular velocity since they are control-independent
    x_init_att_ = x_init_att(:,i);
    attitude_schedule_ = precompute_attitude_profile(att_fun, x_init_att_, steps + predictHolizon*mpc_dt/simulation_dt + 1);
    attitude_schedule(:,:,i) = attitude_schedule_;
end

state_log = state_log__;
input_log = input_log__;
constraint_log = constraint_log__;
index_end = index_end__;

clear state_log__
clear input_log__
clear constraint_log__
clear attitude_schedule_
clear index_end__

save("log/"+windowNum+".mat", "state_log", "input_log","constraint_log", "index_end", "attitude_schedule");

function attitude_schedule = precompute_attitude_profile(att_fun, att_state0, total_steps)
    x_tmp = att_state0;
    attitude_schedule = zeros(7, total_steps);
    for idx = 1:total_steps
        attitude_schedule(:, idx) = x_tmp;
        x_tmp = full(att_fun(x_tmp));
    end
end