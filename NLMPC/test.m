k = 1;
x_current = [1; 2; 3]
q_rtn2body = quaternion(attitude_schedule(1:4,k)');

q = attitude_schedule(1:4,k);
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
rot_h2b * x_current(1:3)