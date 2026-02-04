i = 1;
quaternion_RTN2BODY = quaternion(attitude_schedule(1,1:steps+1, i),attitude_schedule(2,1:steps+1, i),attitude_schedule(3,1:steps+1, i),attitude_schedule(4,1:steps+1, i));
pathRow = state_log(1:3,:,i);
pathXYZ = rotateframe((quaternion_RTN2BODY),pathRow');
lastIndex = index_end(i);

figure('Name',"Trajectory : " + i,'Units','centimeters', 'Position',[0 0 8.5 8.5],'Color','white');
fontsize(10,"points")
[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
plot3(pathXYZ(1:lastIndex,2),pathXYZ(1:lastIndex,1),pathXYZ(1:lastIndex,3),"-b", "LineWidth",2);
hold on
[X, Y, Z] = target.KOZ3D([0 0 1]);
surf(X, Y, Z, "FaceColor", "none","EdgeColor","red","EdgeAlpha",0.2);
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel("$y$ [m]", "Interpreter", "latex")
zlabel("$z$ [m]", "Interpreter", "latex")
legend("Trajectory", "KOZ")
xlim([-30 30])
ylim([-30 30])
zlim([-30 30])
daspect([1 1 1])
