x = linspace(-30, 30, 301);
z = linspace(-30, 30, 301);
[X, Z] = meshgrid(x, z);
phi = zeros(size(X));

index = ceil((indexStampRow(3) + lastIndex)/2);

rho = rhoRow(:, index);
R = [rho(1) rho(2) rho(3);
    0 rho(4) rho(5);
    0 0 rho(6)];
attractiveShapingMatrix = R.'*R;

for i = 1:length(x)
    for j = 1:length(z)
        r_ct = [X(i,j) 0 Z(i,j)].';
        if vecnorm(r_ct ) > 1e-6
            r_cf = r_ct - attractivePose;
            r_cb = target.r_cb(r_ct);
            phi(i,j) = exp(1-r_cb.'*keepOutWidthMatrix*r_cb)*(r_cf.'*keepOutHightMatrix*r_cf);
        end
    end
end

f = figure('Name',"Attractive Potential Field : " + j, 'Units','centimeters', 'Position',[0 0 8 6.5],'Color','white');
contourf(X, Z, phi, 50, 'LineColor', 'none');
colorbar;
hold on;
[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
plot(boundary_RTN(:,1),boundary_RTN(:,2), "Color","red", "DisplayName", "KOZ","LineWidth",1);
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel('z [m]', "Interpreter", "latex")
xlim([-30 30])
ylim([-30 30])
fontsize(8,"points")
daspect([1 1 1])
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';