j = 7;
i = 1;
rhoRow = rho(:,:,j);
pathRow = path(:,:,j);


rho_ = rhoRow(:,i);
R = [rho_(1) rho_(2) rho_(3);
    0 rho_(4) rho_(5);
    0 0 rho_(6)];
attractiveShapingMatrix = R'*R;

quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);
pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';

x = linspace(-30, 30, 6001);
z = linspace(-30, 30, 6001);
[X, Y] = meshgrid(x,z);
phi = zeros(size(X));
parfor m=1:length(x)
    phi_j = phi(:,m);
    position = zeros(3,1);
    position(1) = x(m);
    for k=1:length(z)
        position(2) = 0;
        position(3) = z(k);

        r_ct = position - attractivePose;
        r_cb_ = target.r_cb(position);
        if(r_cb_ == 0)
            phi_j(k) = 0;
        else
            % phi_j(k) = r_ct'*attractiveShapingMatrix*r_ct;
            phi_j(k) = phi_j(k)+exp(1-r_cb_.'*keepOutWidthMatrix*r_cb_)*(r_ct'*keepOutHightMatrix*r_ct);
        end
    end
    phi(:,m) = phi_j;
end

%%
f = figure('unit','centimeter','Position', [0 0 8 7.5],'Color','white');
surf(X,Y,phi, "LineStyle","none");

% c = colorbar;
% c.Label.String = "Potential [m${}^2$/s]";
% c.Label.Interpreter = "latex";
% c.TickLabelInterpreter = "latex";
view([0 0 1])
hold on;
[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
boundary_RTN(:,3) = 30;
b = plot3(boundary_RTN(:,1),boundary_RTN(:,2),boundary_RTN(:,3),Color="red");
pathXYZ(2,:) = 30;
% clim([0 30])
% p = plot3(pathXYZ(1,1:i), pathXYZ(3,1:i),pathXYZ(2,1:i), "LineStyle", "-",Color="k");
% r = scatter3(pathXYZ(1,i), pathXYZ(3,i),pathXYZ(2,i), 10,"filled","o","MarkerFaceColor","k",MarkerEdgeColor = "k");
% legend([b, p], ["KOZ", "Trajecory"], "NumColumns", 2, "Interpreter", "latex","Position",[0.1 0.85, 0.8, 0.1]);
daspect([1,1,1])
fontsize(12,"points")
xlabel('$x$ [m]', 'Interpreter', 'latex')
ylabel('$z$ [m]', 'Interpreter', 'latex')
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
xlim([-30 30])
ylim([-30 30])

%%
f = figure('unit','centimeter','Position', [0 0 11 7.5],'Color','white');
surf(X,Y,phi, "LineStyle","none");
% clim([0 30])
c = colorbar;
c.Label.String = "Potential [m${}^2$/s]";
c.Label.Interpreter = "latex";
c.TickLabelInterpreter = "latex";
% view([0 0 1])
hold on;
[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
boundary_RTN(:,3) = 0;
b = plot3(boundary_RTN(:,1),boundary_RTN(:,2),boundary_RTN(:,3),Color="red");
pathXYZ(2,:) = 30;
% p = plot3(pathXYZ(1,1:i), pathXYZ(3,1:i),pathXYZ(2,1:i), "LineStyle", "-",Color="k");
% r = scatter3(pathXYZ(1,i), pathXYZ(3,i),pathXYZ(2,i), 10,"filled","o","MarkerFaceColor","k",MarkerEdgeColor = "k");
% legend([b, p], ["KOZ", "Trajecory"], "NumColumns", 2, "Interpreter", "latex","Position",[0.1 0.85, 0.8, 0.1]);
daspect([1,1,1])
fontsize(12,"points")
xlabel('$x$ [m]', 'Interpreter', 'latex')
ylabel('$z$ [m]', 'Interpreter', 'latex')
zlabel('Potential [m${}^2$/s]', 'Interpreter','latex')
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.ZAxis.TickLabelInterpreter = 'latex';
xlim([-30 30])
ylim([-30 30])