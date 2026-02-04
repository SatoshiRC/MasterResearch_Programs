close all
clear all

Ncase = 75*21;

DateString = string(datetime('now','Format','MMddHHmmss'));
mkdir("log/"+DateString);

%simulation time vec
simulationTime =1000;
simulation_dt = 0.1;
steps = simulationTime / simulation_dt;
time = linspace(0,simulationTime,steps+1);

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

% Initial condition
x_init = [initialChaserPose'; initialChaserVelo'];
x_init_att = zeros(7, Ncase);
x_init_att(1:4,:) = repmat([0.5; -0.5; -0.5; 0.5],1,Ncase);
att_mat = zeros(3, 75, 21);
att_mat(2,:,:) = repmat(-3.7:0.1:3.7,1,1,21) * pi/180;
att_mat(3,:,:) = repmat(-1:0.1:1,75,1) * pi/180;
x_init_att(5:7,:) = reshape(att_mat,3,Ncase);


%A target is approcimate as rectangle
targetHight = 4; %[m]
targetWidth = 11; %[m]
% A chaser is approximated as cuboid
chaserHight = 0.81;
chaserWidth = 3.7;
chaserDepth = 1.2;
KOZ_r = (sqrt(chaserDepth^2 + chaserWidth^2 + chaserHight^2) + sqrt(targetWidth^2+targetHight^2))/2/(1-exp(-3)); 

%% MPC parameter
nx = 6; %[p_R, p_T, p_N, v_R, v_T, v_N]
nu = 3; %[a_R, a_T, a_N]
np = 7; %[q_w, q_x, q_y, q_z, omega_X, omega_Y, omega_Z]
predictHolizon = 20;
mpc_dt = 5;
mpc_steps = simulationTime / mpc_dt;
x_target = [0, 0, 6.5, 0, 0, 0];

weights.Qp = diag([5, 5, 5]);
weights.Qv = diag([1, 1, 1]);
weights.R = diag([0.1, 0.1, 0.1]);
weights.P = diag([10, 10, 10, 5, 5, 5]);

state_limits = struct();
state_limits.p_R = [-30, 30];
state_limits.p_T = [-30, 30];
state_limits.p_N = [-30, 30];
state_limits.v_R = [-inf, inf];
state_limits.v_T = [-inf, inf];
state_limits.v_N = [-inf, inf];

input_limits = struct();
input_limits.a = [-0.05, 0.05];

constraint = struct();
constraint.approachVelocity = [0, 0.1];
constraint.thrust = [0, 0.05];
constraint.KOZ = [0, inf];

%%
att_fun = attitudeDynamics(simulation_dt, omega, target.InatiaMatrix());
f_sim = make_stateEquation(simulation_dt, omega);
f_mpc = make_stateEquation(mpc_dt, omega);

% Build once on the client to obtain dimension templates for logs and bounds
[solver, base_lbx, base_ubx, lbg, ubg, z0_template] = build_nlmpc_solver( ...
    predictHolizon, nx, nu, np, f_mpc, state_limits, input_limits, ...
    weights, x_target, KOZ_r, constraint);

%% シミュレーション

state_log = zeros(nx, steps+1, Ncase);
input_log = zeros(nu, steps, Ncase);
constraint_log = zeros(length(lbg), steps, Ncase);
index_end = linspace(steps, steps, Ncase);
simEx = mpc_dt / simulation_dt;

outerWaitbar = waitbar(0, "Outer-loop");
innarWaitbar = waitbar(0, "Innar-loop");
for i=1:Ncase
    outerWaitbar = waitbar(i/Ncase, outerWaitbar, "Outer-loop");
% Pre-compute attitude and angular velocity since they are control-independent
x_init_att_ = x_init_att(:,i);
attitude_schedule = precompute_attitude_profile(att_fun, x_init_att_, steps + predictHolizon*mpc_dt/simulation_dt + 1);

x_current = x_init;
state_log_ = state_log(:,:,i);
input_log_ = input_log(:,:,i);
constraint_log_ = constraint_log(:,:,i);
state_log_(:, 1) = x_current;
index_end_ = index_end(i);

% 初期推定軌道（定常）
x_guess = repmat(x_current, predictHolizon+1, 1);
u_guess = zeros(nu*predictHolizon, 1);
p = zeros(np*(predictHolizon+1), 1);

for k = 1:mpc_steps
    innarWaitbar = waitbar(k/mpc_steps, innarWaitbar, "Innar-loop");
    % 境界の更新（現在状態を初期状態に固定）
    lbx = base_lbx;
    ubx = base_ubx;
    lbx(1:nx) = x_current;
    ubx(1:nx) = x_current;

    % Apply pre-computed attitude/omega for each stage within the horizon
    att_index = (k-1)*simEx+1:simEx:(k-1 + predictHolizon)*simEx+1;
    p = reshape(attitude_schedule(:,att_index),np*(predictHolizon+1),1);

    % 初期値ベクトル
    z_init = z0_template;
    z_init(1:nx*(predictHolizon+1)) = x_guess;
    z_init(nx*(predictHolizon+1)+1:end) = u_guess;

    % ソルバ呼び出し
    sol = solver('x0', z_init, 'lbx', lbx, 'ubx', ubx, 'lbg', lbg, 'ubg', ubg, 'p', p);
    z_opt = full(sol.x);
    constraint_log_(:,k) = full(sol.g);

    x_plan = reshape(z_opt(1:nx*(predictHolizon+1)), nx, predictHolizon+1)';
    u_plan = reshape(z_opt(nx*(predictHolizon+1)+1:end), nu, predictHolizon)';

    % 最初の入力を適用
    u_apply = u_plan(1, :)';
    % simとmpcの周波数の違い分繰り返し
    for l = 1:simEx
        kk = (k-1)*simEx + l;
        input_log_(:, kk) = u_apply;
        x_current = full(f_sim(x_current, u_apply));
        state_log_(:, kk+1) = x_current;

        % temporal terminal condition
        q_rtn2body = quaternion(attitude_schedule(1:4,kk+1)');
        if(norm(rotateframe(q_rtn2body, x_current(1:3)')- x_target(1:3)) < 0.1)
            index_end_ = kk;        
            break;
        end
    end

    if(norm(rotateframe(q_rtn2body, x_current(1:3)')- x_target(1:3)) < 0.1)   
        break;
    end

    % 次ステップ用の初期推定をシフト
    x_guess = reshape([x_plan(2:end, :); x_plan(end, :)], [], 1);
    u_guess = reshape([u_plan(2:end, :); u_plan(end, :)], [], 1);
end

save("log/"+DateString+ "/" + DateString + "_" +i+".mat", "state_log_", "input_log_","constraint_log_", "index_end_");

state_log(:,:,i) = state_log_;
input_log(:,:,i) = input_log_;
constraint_log(:,:,i) = constraint_log_;
end

save("log/"+DateString+".mat", "state_log", "input_log","constraint_log", "index_end");

%% 補助関数
function attitude_schedule = precompute_attitude_profile(att_fun, att_state0, total_steps)
    x_tmp = att_state0;
    attitude_schedule = zeros(7, total_steps);
    for idx = 1:total_steps
        attitude_schedule(:, idx) = x_tmp;
        x_tmp = full(att_fun(x_tmp));
    end
end

function [solver, lbx, ubx, lbg, ubg, z0] = build_nlmpc_solver(N, nx, nu, np, f_fun, ...
    state_limits, input_limits, weights, x_target, KOZ_r, constraint)
    import casadi.*

    X = SX.sym('X', nx, N+1);
    U = SX.sym('U', nu, N);
    P = SX.sym('P',np, N+1);

    f = 0;
    g = [];
    lbg = [];
    ubg = [];

    %x_target is given chaser position and velocity in target body frame
    for k = 1:N
        p_rtn_k = X(1:3, k);
        v_rtn_k = X(4:6, k);
        u_k = U(:, k);

        %Get target angular velocity in target body frame
        omega_k = P(5:7, k);

        %Quaternion represents Hill->Body frame rotation
        q = P(1:4, k);
        qw = q(1);
        qx = q(2);
        qy = q(3);
        qz = q(4);

        %Rotation matrix from Hill to Body frame
        rot_h2b = SX.zeros(3,3);
        rot_h2b(1,1) = 1 - 2*(qy^2 + qz^2);
        rot_h2b(1,2) = 2*(qx*qy + qz*qw);
        rot_h2b(1,3) = 2*(qx*qz - qy*qw);
        rot_h2b(2,1) = 2*(qx*qy - qz*qw);
        rot_h2b(2,2) = 1 - 2*(qx^2 + qz^2);
        rot_h2b(2,3) = 2*(qy*qz + qx*qw);
        rot_h2b(3,1) = 2*(qx*qz + qy*qw);
        rot_h2b(3,2) = 2*(qy*qz - qx*qw);
        rot_h2b(3,3) = 1 - 2*(qx^2 + qy^2);

        %Transform position and velocity from Hill to body frame
        p_body_k = rot_h2b * p_rtn_k;
        v_body_k = rot_h2b * v_rtn_k - cross(omega_k, p_body_k);
        
        %Compute errors in body frame
        pos_err = p_body_k - x_target(1:3)';
        vel_err = v_body_k - (x_target(4:6)' - cross(omega_k, x_target(1:3)'));
        
        %Cost function with errors computed in body frame
        f = f + pos_err' * weights.Qp * pos_err + vel_err' * weights.Qv * vel_err + u_k' * weights.R * u_k;

        x_next = f_fun(X(:, k), u_k);
        g = [g; X(:, k+1) - x_next]; %#ok<AGROW>
        lbg = [lbg; zeros(nx, 1)];
        ubg = [ubg; zeros(nx, 1)];

        %KOZ constraint
        rho_sq = dot(p_body_k(1:2), p_body_k(1:2));
        rho = sqrt(rho_sq + 1e-9); 
        theta = atan2(rho, p_body_k(3));
        theta = if_else(theta < 0, -theta, theta);
        r_b_norm = KOZ_r*(1-exp(-theta/(pi/24)));
        r_cb_norm__ = sqrt(dot(p_body_k,p_body_k)) - r_b_norm;
        g = [g; r_cb_norm__];
        lbg = [lbg; 0];
        ubg = [ubg; inf];

        %Approach Velocity Constraint (using body frame)
        v_diff = v_body_k + cross(omega_k, x_target(1:3)');
        pos_err_norm = sqrt(pos_err' * pos_err + 1e-6);
        v_approach   = dot(-pos_err, v_diff) / pos_err_norm;
        v_approach   = if_else(pos_err_norm > 1e-5, v_approach, 0);
        g = [g;v_approach];
        lbg = [lbg; constraint.approachVelocity(1)];
        ubg = [ubg; constraint.approachVelocity(2)];

        %control input constraint
        u_norm = sqrt(u_k'*u_k + 1e-9);
        g = [g; u_norm];
        lbg = [lbg; constraint.thrust(1)];
        ubg = [ubg; constraint.thrust(2)];
    end

    %Terminal cost: transform terminal state to body frame
    q_term = P(1:4, end);
    qw_term = q_term(1);
    qx_term = q_term(2);
    qy_term = q_term(3);
    qz_term = q_term(4);
    
    %Rotation matrix from Hill to body frame for terminal state
    rot_h2b_term = SX.zeros(3,3);
    rot_h2b_term(1,1) = 1 - 2*(qy_term^2 + qz_term^2);
    rot_h2b_term(1,2) = 2*(qx_term*qy_term - qz_term*qw_term);
    rot_h2b_term(1,3) = 2*(qx_term*qz_term + qy_term*qw_term);
    rot_h2b_term(2,1) = 2*(qx_term*qy_term + qz_term*qw_term);
    rot_h2b_term(2,2) = 1 - 2*(qx_term^2 + qz_term^2);
    rot_h2b_term(2,3) = 2*(qy_term*qz_term - qx_term*qw_term);
    rot_h2b_term(3,1) = 2*(qx_term*qz_term - qy_term*qw_term);
    rot_h2b_term(3,2) = 2*(qy_term*qz_term + qx_term*qw_term);
    rot_h2b_term(3,3) = 1 - 2*(qx_term^2 + qy_term^2);
    
    %Terminal target angular velocity
    omega_term = P(5:7, end);

    %Transform terminal position and velocity to body frame
    p_body_term = rot_h2b_term * X(1:3, end);
    v_body_term = rot_h2b_term * X(4:6, end) - cross(omega_term, p_body_term);
    
    %Compute terminal error in body frame
    term_err = [p_body_term; v_body_term] - x_target(1:6)' - [0;0;0;cross(omega_term, x_target(1:3)')];
    f = f + term_err' * weights.P * term_err;

    %Terminal KOZ constraint
    rho_sq = dot(p_body_term(1:2), p_body_term(1:2));
    rho = sqrt(rho_sq + 1e-9); 
    theta = atan2(rho, p_body_term(3));
    theta = if_else(theta < 0, -theta, theta);
    r_b_norm = KOZ_r*(1-exp(-theta/(pi/24)));
    r_cb_norm__ = sqrt(dot(p_body_term,p_body_term)) - r_b_norm;
    g = [g; r_cb_norm__];
    lbg = [lbg; 0];
    ubg = [ubg; inf];

    vars = [reshape(X, nx*(N+1), 1); reshape(U, nu*N, 1)];

    lbx = -inf(size(vars));
    ubx = inf(size(vars));

    % 状態境界（位置・速度のみ制限）
    for k = 1:(N+1)
        offset = (k-1)*nx;
        lbx(offset+1) = state_limits.p_R(1);
        ubx(offset+1) = state_limits.p_R(2);
        lbx(offset+2) = state_limits.p_T(1);
        ubx(offset+2) = state_limits.p_T(2);
        lbx(offset+3) = state_limits.p_N(1);
        ubx(offset+3) = state_limits.p_N(2);
        lbx(offset+4) = state_limits.v_R(1);
        ubx(offset+4) = state_limits.v_R(2);
        lbx(offset+5) = state_limits.v_T(1);
        ubx(offset+5) = state_limits.v_T(2);
        lbx(offset+6) = state_limits.v_N(1);
        ubx(offset+6) = state_limits.v_N(2);
    end

    % 入力境界（各軸共通）
    start_idx = nx*(N+1);
    for k = 1:N
        offset = start_idx + (k-1)*nu;
        lbx(offset+(1:nu)) = input_limits.a(1);
        ubx(offset+(1:nu)) = input_limits.a(2);
    end

    p = reshape(P,np*(N+1),1);

    opts = struct();
    opts.ipopt.print_level = 0;
    opts.ipopt.max_iter = 200;
    opts.ipopt.tol = 1e-6;
    opts.ipopt.hessian_approximation = 'limited-memory';
    opts.print_time = false;

    solver = nlpsol('solver', 'ipopt', struct('x', vars, 'f', f, 'g', g, 'p', p), opts);
    z0 = zeros(size(vars));
end