function f = make_stateEquation(dt, omega)
    arguments
        dt(1,1) double = 0.1
        omega(1,1) double = 0.01
    end
    import casadi.*
    x = SX.sym('x', 6); % [p_R, p_T, p_N, v_R, v_T, v_N]
    u = SX.sym('u', 3); % [a_R, a_T, a_N]
    dstate = SX.zeros(6, 1);

    dstate(1)=x(4);
    dstate(2)=x(5);
    dstate(3)=x(6);
    dstate(4) = u(1)+3*omega^2*x(1)+2*omega*x(5);
    dstate(5) = u(2)-2*omega*x(4);
    dstate(6) = u(3)-omega^2*x(3);

    ode = struct('x',x,'p',u,'ode',dstate);
    ode_opts = struct('tf',dt);
    F = integrator('F','rk',ode,ode_opts);
    res = F('x0',x,'p',u);
    f = Function('f',{x,u},{res.xf});
end