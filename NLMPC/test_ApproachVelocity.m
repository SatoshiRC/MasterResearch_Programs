
approachVelocity = zeros([1,index_end]);
dPose = zeros([1,index_end]);
r_ca = zeros([1,index_end]);
p_body = zeros([3,index_end]);
for k=1:index_end
% for k=15
    if(k == 40)
        disp(k)
    end
    p_rtn_k = state_log(1:3,k);
    v_rtn_k = state_log(4:6,k);
    omega_k = attitude_schedule(5:7, k);

    q = attitude_schedule(1:4, k);
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    rot_h2b(1,1) = 1 - 2*(qy^2 + qz^2);
    rot_h2b(1,2) = 2*(qx*qy + qz*qw);
    rot_h2b(1,3) = 2*(qx*qz - qy*qw);
    rot_h2b(2,1) = 2*(qx*qy - qz*qw);
    rot_h2b(2,2) = 1 - 2*(qx^2 + qz^2);
    rot_h2b(2,3) = 2*(qy*qz + qx*qw);
    rot_h2b(3,1) = 2*(qx*qz + qy*qw);
    rot_h2b(3,2) = 2*(qy*qz - qx*qw);
    rot_h2b(3,3) = 1 - 2*(qx^2 + qy^2);

    p_body_k = rot_h2b * p_rtn_k;
    v_body_k = rot_h2b * v_rtn_k - cross(omega_k, p_body_k);
    pos_err = p_body_k - x_target(1:3)';

    v_diff = v_body_k + cross(omega_k, x_target(1:3)');
    pos_err_norm = sqrt(pos_err' * pos_err + 1e-6);
    approachVelocity(k)   = dot(-pos_err, v_body_k) / pos_err_norm;

        q = attitude_schedule(1:4, k+1);
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);
    rot_h2b(1,1) = 1 - 2*(qy^2 + qz^2);
    rot_h2b(1,2) = 2*(qx*qy + qz*qw);
    rot_h2b(1,3) = 2*(qx*qz - qy*qw);
    rot_h2b(2,1) = 2*(qx*qy - qz*qw);
    rot_h2b(2,2) = 1 - 2*(qx^2 + qz^2);
    rot_h2b(2,3) = 2*(qy*qz + qx*qw);
    rot_h2b(3,1) = 2*(qx*qz + qy*qw);
    rot_h2b(3,2) = 2*(qy*qz - qx*qw);
    rot_h2b(3,3) = 1 - 2*(qx^2 + qy^2);
    dPose(k) = (norm(p_body_k - x_target(1:3)') - norm(rot_h2b*state_log(1:3,k+1) - x_target(1:3)'))/0.1;
    r_ca(k) = norm(p_body_k - x_target(1:3)');
    p_body(:,k) = p_body_k;

end

figure('Name', 'approach velocity')
hold on
plot(time(1:index_end), approachVelocity(1, 1:index_end));
plot(time(1:index_end), dPose(1, 1:index_end));
plot(time(1:simEx:index_end), constraint_log(8, 1:ceil(index_end/simEx)),'LineStyle', '--');
legend("Simulate", "dPose ","MPC result")
xlabel("time [s]")
ylabel("approach velocity [m/s]")

figure('Name', 'r_{ca}')
hold on
plot(time(1:index_end), r_ca(1, 1:index_end));
% plot(time(1:simEx:index_end), constraint_log(8, 1:ceil(index_end/simEx)),'LineStyle', '--');
legend("Simulate")
xlabel("time [s]")
ylabel("r_ca [m]")

figure('Name', 'Trajectory')
plot3(p_body(1,:),p_body(2,:),p_body(3,:))
daspect([1,1,1]);