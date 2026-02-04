
figure('Name',"Trajectory : " + j,'Units','centimeters', 'Position',[0 0 10 10],'Color','white');

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
    p = plot(pathXYZ(1, 1:lastIndex),pathXYZ(3, 1:lastIndex),"-k", "LineWidth",2);
    hold on
end

[boundary_RTN]= target.koz([0 0 1], [0 1 0]);
b = plot(boundary_RTN(:,1),boundary_RTN(:,2),Color="red");

fontsize(10,"points")
xlabel("$x$ [m]", "Interpreter", "latex")
ylabel("$z$ [m]", "Interpreter", "latex")
legend([p, b],["Path", "KOZ"]);
xlim([-30 30])
ylim([-30 30])
daspect([1 1 1])
fontsize(12,"points")
