
r_cb_norm = zeros([1,index_end]);
r_cb_norm2 = zeros([1,index_end]);
for k=1:index_end
% for k=15
    p_rtn_k = state_log(1:3,k);

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
    pos_err = p_body_k - x_target(1:3)';

    rho_sq = dot(p_body_k(1:2), p_body_k(1:2));
    rho = sqrt(rho_sq + 1e-9); 
    theta = atan2(rho, p_body_k(3));
    if theta > 0
        theta = theta;
    else
        theta = -theta;
    end
    r_b_norm = KOZ_r*(1-exp(-theta/(pi/24)));
    r_cb_norm(k) = sqrt(dot(p_body_k,p_body_k)) - r_b_norm;

    q = quaternion(attitude_schedule(1:4,k)');
    p_body_k = rotateframe(q, p_rtn_k')';
    theta = atan2(sqrt(dot(p_body_k(1:2),p_body_k(1:2))), p_body_k(3));
    if theta > 0
        theta = theta;
    else
        theta = -theta;
    end
    r_b_norm = KOZ_r*(1-exp(-theta/(pi/24)));
    r_cb_norm2(k) = sqrt(dot(p_body_k, p_body_k)) - r_b_norm;

end

figure('Name', 'R_cb')
hold on
plot(time(1:index_end), r_cb_norm(1, 1:index_end));
plot(time(1:index_end), r_cb_norm2(1, 1:index_end));
plot(time(1:index_end), constraint_log(7, 1:index_end),'LineStyle', '--');
legend("Simulate", "simulate2", "MPC result")
xlabel("time [s]")
ylabel("r_cb [m]")