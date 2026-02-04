for i=1:3
    figure('Name',"position",'unit','centimeter','Position', [0 0 12 6],'Color','white')
    hold on
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
        plot(time(1:lastIndex),pathXYZ(i,1:lastIndex));
    end
    xlabel('Time [s]')
    ylabel('Position [m]')
    xlim([0 450])
    % leg = legend("$r_{ct, x}$","$r_{ct, z}$");
    leg.FontSize = 8;
    leg.Interpreter = "latex";
    fontsize(8,"points")
    grid on
end