
pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.');

f = figure('Name',"Trajectory : " + j,'Units','centimeters', 'Position',[0 0 8 8],'Color','white');
% figure('Name',"Trajectory : " + j,'Units','centimeters', 'Position',[0 0 13.5 13.5],'Color','white');
[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
hold off
plot(pathXYZ(indexStampRow(1)+1:indexStampRow(2),1),pathXYZ(indexStampRow(1)+1:indexStampRow(2),3), "LineWidth",1, "LineStyle","--", "Color",[0 0 0]/255, "DisplayName", "Phase 1");
hold on
plot(pathXYZ(indexStampRow(2)+1:lastIndex,1),pathXYZ(indexStampRow(2)+1:lastIndex,3),"Color","blue", "LineStyle","-", "LineWidth",1, "DisplayName", "Phase 2");
plot(boundary_RTN(:,1),boundary_RTN(:,2), "Color","red", "DisplayName", "KOZ");
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel('z [m]', "Interpreter", "latex")
legend("Position", [0.60 0.66 0.2 0.2], "Interpreter", "latex")
fontsize(12,"points")
xlim([-30 30])
ylim([-30 30])
zlim([-30 30])
daspect([1 1 1])
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';