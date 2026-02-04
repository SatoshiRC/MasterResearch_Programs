
f = figure('Name',"Trajectories" ,'Units','centimeters', 'Position',[0 0 9 8],'Color','white');

list = [24 25 26];
lineColors = ["k","#00C850","b"];
for i=1:length(list)
    j = list(i);
    quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);
    pathRow = path(:,:,j);
    indexStampRow = indexStamp(:,j);
    if isArrived(j) > 0
        lastIndex = isArrived(j)-1;
    elseif isArrived(j) < 0
        lastIndex = -isArrived(j)-1;
    else
        lastIndex = steps-1;
    end
    if indexStampRow(3) == 0
        indexStampRow(3) = lastIndex;
    end

    pathXYZ = rotateframe(conj(quaternion_BODY2RTN_),pathRow.').';
    plot3(pathXYZ(2, 1:lastIndex),pathXYZ(1, 1:lastIndex),pathXYZ(3, 1:lastIndex),"-", "LineWidth",2, "DisplayName", "l="+j, "Color", lineColors(i));
    hold on
end


[X, Y, Z] = target.KOZ3D([0 0 1]);
surf(X, Y, Z, "FaceColor", "none","EdgeColor","red","EdgeAlpha",0.2, "DisplayName", "KOZ");

legend("Interpreter", 'latex','Position',[0.7 0.65 0.25 0.2])

fontsize(12,"points")
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel("$y$ [m]", "Interpreter", "latex")
zlabel("$z$ [m]", "Interpreter", "latex")
xlim([-30 30])
ylim([-30 30])
zlim([-30 30])
daspect([1 1 1])
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.ZAxis.TickLabelInterpreter = 'latex';
