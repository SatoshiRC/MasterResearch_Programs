% This shows the guidance and control simulation using Lyapunov Vector Field (LVF)

% Target Specifications
D_0 = [6.5; 0; 0];
omega = [0; 0; -2.6*pi/180];
O_0 = [1; 0; 0];
O_0 = O_0/norm(O_0);
theta_D = 15*pi/180;
phi_0 = -0/6;
quat = quaternion(cos(phi_0/2), 0, 0, sin(phi_0/2));

%chaser state
r_i = [30; 0; 0];
v_i = [0; 0; 0];

% Simulation Parameters
dt = 0.01; % time step
T = 2000; % total time
N = T/dt; % number of steps
t = linspace(0, T, N+1);
r = zeros(3, N+1);
r_hill = zeros(3, N+1);
r_D_ = zeros(3, N+1);
v = zeros(3, N+1);
q = zeros(4, N+1);
theta = zeros(1, N);
r(:,1) = r_i;
v(:,1) = v_i;
q(:,1) = quat.compact();
r_hill(:,1) = rotateframe(quat, r_i')';
a_max = 0.05;
v_max = 0.2;

a = norm(omega)^2;
b = norm(omega)^2*norm(D_0)+2*norm(omega)*norm(v_max)-a_max;
c = v_max^2*2;
A_max = (-b+sqrt(b^2-4*a*c))/(2*a);
if a == 0
    A_min = v_max^2*2/a_max;
else
    A_min = (-b-sqrt(b^2-4*a*c))/(2*a);
end
% A = 8.25;
A = 14.75;

if or(A<A_min, A<A_min)
    print("A error")
    return
end

w = norm(omega);
q_dot = quaternion([0; omega]')/2;
i_end = N+1;

figure("Name","Animation");
for i = 1:N
    % Calculate the desired position and orientation
    D = rotateframe(quat, D_0')';
    O = rotateframe(quat, O_0')';
    
    r_D = r(:,i) - D;
    if norm(r_D) < 0.1
        i_end = i;
    end
    r_D_(:,i) = r(:,i) - D;

    theta_ = acos(dot(O, r_D) / (norm(O) * norm(r_D)));
    
    if theta_ < theta_D
        theta_n = pi*theta_/(2*theta_D);
    else
        theta_n = pi/2;
    end
    cThetaD = (-12*theta_D^2 + 4*pi*theta_D+pi^2)/(4*theta_D^2);
    if theta_D <= pi/2
        f_thetaD = sqrt(cThetaD + 4);
    else
        f_thetaD = 2;
    end

    Gamma = eye(3) / norm(r_D);
    C = -(r_D'*Gamma)';
    S = cross(r_D,cross(O, r_D)) / norm(cross(r_D,cross(O, r_D)));
    % Calculate the Lyapunov Vector Field (LVF)
    v_ref = v_max*(norm(r_D)/A)^2;
    if norm(r_D) > A
        h = -v_max*r_D/norm(r_D);
    else
        % h = rotateframe(quat, v_ref*(C*cos(theta_n) + S*sin(theta_n))')'-cross(rotateframe(quat, omega')', r(:,i));
        h = v_ref*(C*cos(theta_n) + S*sin(theta_n))-cross(rotateframe(quat, omega')', r(:,i));
    end
    % Update the chaser state
    r(:,i+1) = r(:,i) + v(:,i) * dt;
    v(:,i+1) = h;
    theta(1,i) = theta_;
    quat = quat + quat*q_dot*dt;
    q(:,i+1) = quat.compact();
    r_hill(:,i+1) = rotateframe(conj(quat), r(:,i+1)')';

    % if mod(i,100) == 0
    %     hold off;
    %     scatter(r(1,i+1), r(2,i+1), 'filled');
    %     hold on;
    %     quiver(0,0,D(1), D(2), 10, "filled","Color",'k')
    %     quiver(D(1), D(2), O(1), O(2), 10, "filled","Color",'k')
    %     scatter(D(1), D(2), 'k', 'LineWidth', 2);
    %     viscircles(D(1:2)', A, 'Color', 'r', 'LineStyle', '-');
    %     plot(r(1,1:i), r(2,1:i), 'b', 'LineWidth', 2);
    %     daspect([1 1 1]);
    %     drawnow;
    % end
end

figure("Name","LVF Guidance and Control");
hold on;
% viscircles(D(1:2)', A, 'Color', 'r', 'LineStyle', '-');
plot(r(1,1:i_end), r(2,1:i_end), 'b', 'LineWidth', 2);
daspect([1 1 1]);
xlim([-40 40]);
ylim([-40 40]);

figure("Name","Theta")
hold on;
plot(t(1:N),theta);

%%
figure("Name","r")
plot(t(1:N+1),vecnorm(r,2));
%%
f = figure("Name","Distance",'Units','centimeters', 'Position',[0 0 13.5 7],'Color','white');
plot(t(1:N+1),vecnorm(r_D_,2), "Color","k", "DisplayName", "KOZ");
fontsize(12,"points")
% legend("Location", "northeast", "Interpreter", "latex")
ylabel("distance [m/s]", "Interpreter", "latex")
xlabel("time [s]", "Interpreter", "latex")
hold on
grid on
plot([255,400], [14.7, 0], "Color", "red", "LineStyle","--")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';

%%
f = figure("Name","velocity");
plot(t(1:N+1),vecnorm(v,2), "Color","k", "DisplayName", "KOZ");
fontsize(12,"points")
% legend("Location", "northeast", "Interpreter", "latex")
ylabel("velocity [m/s]", "Interpreter", "latex")
xlabel("time [s]", "Interpreter", "latex")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
%%
% figure("Name", "quaternion")
% hold on;
% plot(t(1:N+1), q(1,:));
% plot(t(1:N+1), q(2,:));
% plot(t(1:N+1), q(3,:));
% plot(t(1:N+1), q(4,:));
% legend
f = figure("Name", "Trajectory",'Units','centimeters', 'Position',[0 0 13.5 13.5],'Color','white')
hold on
box on
plot(r_hill(2,:), r_hill(1,:),"Color","k", "LineStyle","-", "LineWidth",1, "DisplayName", "Trajectory")
target = target_KOZ1();
koz = target.koz([0 0 1], [0 1 0]);
plot(koz(:,1), koz(:,2), "Color","red", "DisplayName", "KOZ")
xlim([-30 30])
ylim([-30 30])
daspect([1 1 1]);
fontsize(12,"points")
legend("Location", "northeast", "Interpreter", "latex")
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel("$z$ [m]", "Interpreter", "latex")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
