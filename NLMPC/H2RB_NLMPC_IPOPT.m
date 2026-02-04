close all
clear all

Ncase = 1;

%simulation time vec
simulationTime =300;
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

x_init = [initialChaserPose'; initialChaserVelo'];
x_init_att = [0.5; -0.5; -0.5; 0.5; 0; 2.05*pi/180; 0];

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
mpc_dt = 3;
mpc_steps = simulationTime / mpc_dt;
x_target = [0, 0, 6.5, 0, 0, 0];

weights.Qp = diag([2, 2, 1]);
weights.Qv = diag([1, 1, 1]);
weights.R = diag([2, 2, 2]);
weights.P = diag([1, 1, 1, 1, 1, 1]);

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
constraint.approachVelocity = [-inf, 0.1];
constraint.thrust = [0, 0.05];
constraint.KOZ = [0, inf];

%%
angularVeloRandom = zeros([Ncase 3]);
x0 = zeros([Ncase 10]);

%%
f_sim = make_stateEquation(simulation_dt, omega);
f_mpc = make_stateEquation(mpc_dt, omega);
att_fun = attitudeDynamics(simulation_dt, omega, target.InatiaMatrix());
[solver, base_lbx, base_ubx, lbg, ubg, z0_template] = build_nlmpc_solver( ...
    predictHolizon, nx, nu, np, f_mpc, state_limits, input_limits, weights, x_target, KOZ_r, constraint);

% Pre-compute attitude and angular velocity since they are control-independent
attitude_schedule = precompute_attitude_profile(att_fun, x_init_att, steps + predictHolizon*mpc_dt/simulation_dt + 1);
%%
%% シミュレーション
x_current = x_init;
state_log = zeros(nx, steps+1);
input_log = zeros(nu, steps);
state_log(:, 1) = x_current;
constraint_log = zeros(length(lbg), steps);

% 初期推定軌道（定常）
x_guess = repmat(x_current, predictHolizon+1, 1);
u_guess = zeros(nu*predictHolizon, 1);
p = zeros(np*(predictHolizon+1), 1);

index_end = steps;
simEx = mpc_dt / simulation_dt;
for k = 1:mpc_steps
    fprintf('Step %02d | current pos = (%.2f, %.2f)\n', k, x_current(1), x_current(2));

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
    constraint_log(:,k) = full(sol.g);

    x_plan = reshape(z_opt(1:nx*(predictHolizon+1)), nx, predictHolizon+1)';
    u_plan = reshape(z_opt(nx*(predictHolizon+1)+1:end), nu, predictHolizon)';

    % 最初の入力を適用
    u_apply = u_plan(1, :)';
    % simとmpcの周波数の違い分繰り返し
    for l = 1:simEx
        kk = (k-1)*simEx + l;
        input_log(:, kk) = u_apply;
        x_current = full(f_sim(x_current, u_apply));
        state_log(:, kk+1) = x_current;

        % temporal terminal condition
        q_rtn2body = quaternion(attitude_schedule(1:4,kk+1)');
        if(norm(rotateframe(q_rtn2body, x_current(1:3)')- x_target(1:3)) < 0.1)
            index_end = kk;        
            break;
        end
    end

    if(norm(rotateframe(q_rtn2body, x_current(1:3)')- x_target(1:3)) < 0.1)   
        break;
    end

    % 次ステップ用の初期推定をシフト
    x_guess = reshape([x_plan(2:end, :); x_plan(end, :)], [], 1);
    % for stage = 1:(predictHolizon+1)
    %     idx = (stage-1)*nx;
    %     att_idx = min(k + stage, size(attitude_schedule, 2));
    %     x_guess(idx+7:idx+13) = attitude_schedule(:, att_idx);
    % end
    u_guess = reshape([u_plan(2:end, :); u_plan(end, :)], [], 1);
end

%%
DateString = string(datetime('now','Format','MMddHHmmss'));
save("log/"+DateString+".mat")

%% plot
figure('Name', 'NLMPC Sample');
subplot(3,1,1);
hold on;
plot(time(1:index_end+1), state_log(1, 1:index_end+1))
plot(time(1:index_end+1), state_log(2, 1:index_end+1))
plot(time(1:index_end+1), state_log(3, 1:index_end+1))
legend("r","t","n")
ylabel("Position [m]")
xlabel("Time [s]")

subplot(3,1,2);
hold on;
plot(time(1:index_end+1), state_log(4, 1:index_end+1))
plot(time(1:index_end+1), state_log(5, 1:index_end+1))
plot(time(1:index_end+1), state_log(6, 1:index_end+1))
legend("r","t","n")
ylabel("Velocity [m/s]")
xlabel("Time [s]")

subplot(3,1,3);
hold on;
plot(time(1:index_end), input_log(1, 1:index_end))
plot(time(1:index_end), input_log(2, 1:index_end))
plot(time(1:index_end), input_log(3, 1:index_end))
legend("r","t","n")
ylabel("Thrust [N]")
xlabel("Time [s]")

path_body = rotateframe(quaternion(attitude_schedule(1:4,1:index_end+1)'),state_log(1:3,1:index_end+1)')';
figure('Name','NLMPC Trajectory')
plot3(path_body(1,:),path_body(2,:),path_body(3,:))
hold on
[X, Y, Z] = target.KOZ3D([0 0 1]);
surf(X, Y, Z, "FaceColor", "none","EdgeColor","red","EdgeAlpha",0.2);
xlabel("x [m]")
ylabel("y [m]")
zlabel("z [m]")
xlim([-30,30])
ylim([-30,30])
zlim([-30,30])
daspect([1 1 1])

figure('Name', 'Path log');
hold on;
plot(time(1:index_end+1), path_body(1, 1:index_end+1))
plot(time(1:index_end+1), path_body(2, 1:index_end+1))
plot(time(1:index_end+1), path_body(3, 1:index_end+1))
legend("r","t","n")
ylabel("Position [m]")
xlabel("Time [s]")

%%
figure('Name', 'Constraint')
subplot(3,1,1)
plot(time(1:simEx:index_end), constraint_log(7, 1:ceil(index_end/simEx)));
subplot(3,1,2)
plot(time(1:simEx:index_end), constraint_log(8, 1:ceil(index_end/simEx)));
subplot(3,1,3)
plot(time(1:simEx:index_end), constraint_log(9, 1:ceil(index_end/simEx)));


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

    solver = nlpsol('solver', 'ipopt', struct('x', vars, 'f', f, 'g', g, 'p', p), opts);
    z0 = zeros(size(vars));
end