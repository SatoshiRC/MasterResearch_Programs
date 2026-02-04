pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.');

figure('Name',"Trajectory : " + j,'Units','centimeters', 'Position',[0 0 8.5 8.5],'Color','white');
fontsize(10,"points")
[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
hold off
plot3(pathXYZ(1:indexStampRow(1),2),pathXYZ(1:indexStampRow(1),1),pathXYZ(1:indexStampRow(1),3), "LineWidth",2, "Color",[237 125 49]/255);
hold on
plot3(pathXYZ(indexStampRow(1)+1:indexStampRow(2),2),pathXYZ(indexStampRow(1)+1:indexStampRow(2),1),pathXYZ(indexStampRow(1)+1:indexStampRow(2),3), "LineWidth",2, "Color",[0 176 80]/255);
plot3(pathXYZ(indexStampRow(2)+1:lastIndex,2),pathXYZ(indexStampRow(2)+1:lastIndex,1),pathXYZ(indexStampRow(2)+1:lastIndex,3),"-b", "LineWidth",2);
[X, Y, Z] = target.KOZ3D([0 0 1]);
surf(X, Y, Z, "FaceColor", "none","EdgeColor","red","EdgeAlpha",0.2);
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel("$y$ [m]", "Interpreter", "latex")
zlabel("$z$ [m]", "Interpreter", "latex")
legend("Trajectory(phase 1)", "Trajectory(phase 2)", "Trajectory(phase 3)", "KOZ")
xlim([-30 30])
ylim([-30 30])
zlim([-30 30])
daspect([1 1 1])
