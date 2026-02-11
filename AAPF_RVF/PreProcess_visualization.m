j=1;

quaternion_BODY2RTN_ = quaternion_BODY2RTN(:,j);
angularVelocityRow = targetAngularVelocity(:,:,j);
rhoRow = rho(:,:,j);
pathRow = path(:,:,j);
veloRow = velo(:,:,j);
indexStampRow = indexStamp(:,j);
disp(angularVelocityRow(1,:)*180/pi)

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

