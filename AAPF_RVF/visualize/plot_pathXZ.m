
pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.');
% figure('Name',"path_ZR : " + j);
% for i=1:lastIndex
%     if mod(i,100) == 1
%         [boundary_RTN]= target.koz([0 0 1], [0 1 0]);
%         hold off
%         plot(pathZR(1:i,2),pathZR(1:i,1))
%         hold on
%         plot(boundary_RTN(:,1),boundary_RTN(:,2),Color="red")
%         xlabel('R[m]')
%         ylabel('z [m]')
%         xlim([0 30])
%         drawnow;
%     end
% end

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
% title("Chaser Trajectory (No."+j+")")
% legend("Trajectory(phase 1)", "Trajectory(phase 2)", "Trajectory(phase 3)", "KOZ")
legend("Position", [0.63 0.66 0.2 0.2], "Interpreter", "latex")
% legend("Location","northeast")
fontsize(12,"points")
xlim([-30 30])
ylim([-30 30])
zlim([-30 30])
daspect([1 1 1])
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';