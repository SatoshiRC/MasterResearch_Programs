jj = 25;
% figure('Name',"Trajectory : " + j,'Units','centimeters', 'Position',[0 0 15 12],'Color','white');
f = figure('Name',"Trajectory : " + j,'Units','centimeters', 'Position',[0 0 9 9],'Color','white');
load("Case4/proposedMethod.mat");
j = jj;
quaternion_BODY2RTN = quaternion_BODY2RTN(:,j);
pathXYZ = rotateframe(conj(quaternion_BODY2RTN),path(:,:,j).');
lastIndex = isArrived(j);
plot3(pathXYZ(1:lastIndex,2),pathXYZ(1:lastIndex,1),pathXYZ(1:lastIndex,3),"-", "LineWidth",2, "Color", [0 200 80]/255);
hold on
load("Case4/conventionalMethod.mat");
j = jj;
quaternion_BODY2RTN = quaternion_BODY2RTN(:,j);
pathXYZ = rotateframe(conj(quaternion_BODY2RTN),path(:,:,j).');
lastIndex = isArrived(j);
if lastIndex == 0
    lastIndex = steps;
end
plot3(pathXYZ(1:lastIndex,2),pathXYZ(1:lastIndex,1),pathXYZ(1:lastIndex,3),"-", "LineWidth",1.3, "Color","k");
[X, Y, Z] = target.KOZ3D([0 0 1]);
surf(X, Y, Z, "FaceColor", "none","EdgeColor","red","EdgeAlpha",0.4, "LineWidth", 0.1);
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel("$y$ [m]", "Interpreter", "latex")
zlabel("$z$ [m]", "Interpreter", "latex")
xlim([-30 30])
ylim([-30 30])
zlim([-30 30])
daspect([1 1 1])
fontsize(12,"points")
legend("AAPF + RVF", "AAPF", "KOZ", NumColumns=3, Interpreter='latex', Position = [0.18 0.8 0.65 0.08])
view(-127.5, 30)
fontsize(12,"points");
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.ZAxis.TickLabelInterpreter = 'latex';
