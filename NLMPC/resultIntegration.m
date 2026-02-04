clear all
directory = "log/sim4/";
resFiles = ["1.mat";"0108000556.mat";"3.mat";];

Ncase = 75*21;
simulationTime =1000;
simulation_dt = 0.1;
steps = simulationTime / simulation_dt;

nx = 6; %[p_R, p_T, p_N, v_R, v_T, v_N]
nu = 3; %[a_R, a_T, a_N]
np = 7; %[q_w, q_x, q_y, q_z, omega_X, omega_Y, omega_Z]

load(directory+ resFiles(1));
state_log__ = state_log;
input_log__ = input_log;
constraint_log__ = constraint_log;
attitude_schedule_ = attitude_schedule;
index_end__ = index_end;
for i=2:3
    wind = 525;
    activeRange = wind*(i-1)+1:1:wind*i;

    load(directory+ resFiles(i), "state_log");
    state_log__(:,:,activeRange) = state_log(:,:,activeRange);
    load(directory+ resFiles(i), "input_log");
    input_log__(:,:,activeRange) = input_log(:,:,activeRange);
    load(directory+ resFiles(i), "constraint_log");
    constraint_log__(:,:,activeRange) = constraint_log(:,:,activeRange);
    % load(directory+ resFiles(i), "attitude_schedule");
    % attitude_schedule_(:,:,activeRange) = attitude_schedule(:,:,activeRange);
    load(directory+ resFiles(i), "index_end");
    index_end__(activeRange) = index_end(activeRange);
end

%%
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
predictHolizon = 20;
mpc_dt = 3;

att_fun = attitudeDynamics(simulation_dt, omega, target.InatiaMatrix());
attitude_schedule = zeros(np, steps + predictHolizon*mpc_dt/simulation_dt + 1, Ncase);
for i=activeRange    
    % Pre-compute attitude and angular velocity since they are control-independent
    x_init_att_ = x_init_att(:,i);
    attitude_schedule_ = precompute_attitude_profile(att_fun, x_init_att_, steps + predictHolizon*mpc_dt/simulation_dt + 1);
    attitude_schedule(:,:,i) = attitude_schedule_;
end
clear attitude_schedule_

state_log = state_log__;
input_log = input_log__;
constraint_log = constraint_log__;
% attitude_schedule = attitude_schedule_;
index_end = index_end__;

clear state_log__
clear input_log__
clear constraint_log__
clear attitude_schedule_
clear index_end__

function attitude_schedule = precompute_attitude_profile(att_fun, att_state0, total_steps)
    x_tmp = att_state0;
    attitude_schedule = zeros(7, total_steps);
    for idx = 1:total_steps
        attitude_schedule(:, idx) = x_tmp;
        x_tmp = full(att_fun(x_tmp));
    end
end