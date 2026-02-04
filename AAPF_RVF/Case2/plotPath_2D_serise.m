f =figure('Name',"Trajectory : " + j,'Units','centimeters', 'Position',[0 0 22 9],'Color','white');

for j=1:Ncase
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
    if j == 7
        p7 = plot(pathXYZ(1, 1:lastIndex),pathXYZ(3, 1:lastIndex),"-", "LineWidth",2, "SeriesIndex",1);
    else
        p = plot(pathXYZ(1, 1:lastIndex),pathXYZ(3, 1:lastIndex),"-k", "LineWidth",1.5);
    end
    hold on
end

ip = scatter(0, 30,30,"Marker",'o', 'MarkerEdgeColor','k', "MarkerFaceColor",'k');

[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
b = plot(boundary_RTN(:,1),boundary_RTN(:,2),Color="red");

% fontsize(10,"points")
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel("$z$ [m]", "Interpreter", "latex")
legend([ip, p, p7, b],["Initial Point", "Trajectory $(l=1,\cdots, 6)$", "Trajectory $(l=7)$" "KOZ"],"Position",[0.64 0.6 0.28 0.3],"Interpreter","latex");
% legend([ip, p, p7, b],["Initial Point", "Trajectory $(l=1,\cdots, 6)$", "Trajectory $(l=7)$" "KOZ"],"Location",'eastoutside',"Interpreter","latex");
xlim([-31 10])
ylim([-31 31])
daspect([1 1 1])
fontsize(12,"points")
f.CurrentAxes.XAxis.TickLabelInterpreter = 'latex';
f.CurrentAxes.YAxis.TickLabelInterpreter = 'latex';