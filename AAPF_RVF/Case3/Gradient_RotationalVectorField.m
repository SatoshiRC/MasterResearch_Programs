x = linspace(-30, 30, 21);
z = linspace(-30, 30, 21);
[X, Z] = meshgrid(x, z);
vector = zeros([size(X) 3]);

lastIndex = isArrived(1);
index = ceil((indexStampRow(2) + lastIndex)/2);

rhoRow_r = rho_r(:,:,1);
rho = rhoRow_r(:, index);
R = [rho(1) rho(2) rho(3);
    0 rho(4) rho(5);
    0 0 rho(6)];
attractiveShapingMatrix = R.'*R;

for i = 1:length(x)
    for j = 1:length(z)
        r_ct = [X(i,j) 0 Z(i,j)].';
        w = cross(cross(r_ct, attractivePose),r_ct);
        vector(i,j,:) = attractiveShapingMatrix * w;
    end
end

f = figure('Name',"Rotational Vector Field", 'Units','centimeters', 'Position',[0 0 8 8],'Color','white');
quiver(X,Z,vector(:,:,1),vector(:,:,3),'Color',[0 0 0]/255);
% colorbar;
hold on;
[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
plot(boundary_RTN(:,1),boundary_RTN(:,2), "Color","red", "DisplayName", "KOZ");
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel('z [m]', "Interpreter", "latex")
fontsize(12,"points")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
daspect([1 1 1])