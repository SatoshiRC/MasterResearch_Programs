function f = attitudeDynamics(dt, omega_orbit, InartiaMat)
    arguments
        dt(1,1) double = 0.1
        omega_orbit(1,1) double = 0.01
        InartiaMat(3,3) double = [[1 0 0];[0 1 0];[0 0 1]]
    end
    import casadi.*

    att = SX.sym('att', 7); % [q_w, q_x, q_y, q_z, omega_X, omega_Y, omega_Z]
    q = att(1:4);
    omega_body = att(5:7);
    datt = SX.zeros(7, 1);

    % gravity-gradient induced angular acceleration
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
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

    % omega_body_rtn = rot_h2b * omega_body;
    % quaternion derivative
    skewOmega = [      0,        omega_body(3),  -omega_body(2);
                -omega_body(3),             0,   omega_body(1);
               omega_body(2),  -omega_body(1),              0];
    OmegaMat = [0, -omega_body.';
                omega_body, skewOmega];
    dq = 0.5 * OmegaMat * q;
    datt(1:4) = dq;

    radial_rtn = [1; 0; 0];
    radial_body = rot_h2b * radial_rtn;

    Iomega = InartiaMat * omega_body;
    omega_cross_Iomega = [omega_body(2)*Iomega(3) - omega_body(3)*Iomega(2);
                          omega_body(3)*Iomega(1) - omega_body(1)*Iomega(3);
                          omega_body(1)*Iomega(2) - omega_body(2)*Iomega(1)];
    Ir = InartiaMat * radial_body;
    torque_gg = 3*omega_orbit^2 * [radial_body(2)*Ir(3) - radial_body(3)*Ir(2);
                                    radial_body(3)*Ir(1) - radial_body(1)*Ir(3);
                                    radial_body(1)*Ir(2) - radial_body(2)*Ir(1)];
    domega = InartiaMat\(-omega_cross_Iomega) - skewOmega*rot_h2b*[0 0 omega_orbit]';
    datt(5:7) = domega;

    ode = struct('x', att, 'ode', datt);
    ode_opts = struct('tf', dt);
    F = integrator('F', 'rk', ode, ode_opts);
    res = F('x0', att);
    f = Function('f', {att}, {res.xf});
end
