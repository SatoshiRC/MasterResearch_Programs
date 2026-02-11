tic
for j=1:Ncase
% for j = 2202
    R = R_matrix_0;
    rho_ = [R_matrix_0(1,1) 0 0 R_matrix_0(2,2) 0 R_matrix_0(3,3)];
    rho_dot_ = zeros(size(rho_));

    aapf = Aapf(rho_, rho_dot_, rhoUpper, rhoUnder, Ka, uLim, vLim, dt);
    
    R_r = R_matrix_r_0;
    rho_r_ = [R_matrix_r_0(1,1) 0 0 R_matrix_r_0(2,2) 0 R_matrix_r_0(3,3)];
    rho_dot_r_ = zeros(size(rho_r_));
    rvf = Rvf(rho_r_, rho_dot_r_, rhoUpper_r, rhoUnder_r, Krotation, uLim, vLim, dt);


    approachSim = ApproachSim();
    %-----------constant params-----------------%
    approachSim.uLim = uLim;
    approachSim.vLim = vLim;
    approachSim.steps = steps;
    approachSim.dt = dt;
    approachSim.attractivePose = attractivePose;
    approachSim.Ku = Ku;
    approachSim.orbitalAngularVelocity = omega;
    %-----------pre-calculated variables--------%
    approachSim.quaternion_BODY2RTN = quaternion_BODY2RTN(:,j);
    approachSim.quaternion_ECI2BODY = quaternion_ECI2BODY(:,j);
    approachSim.quaternion_ECI2RTN = quaternion_ECI2RTN;
    approachSim.targetAngularVeloXYZ = targetAngularVelocity(:,:,j);
    approachSim.targetAngularVeloRTN = rotateframe(approachSim.quaternion_BODY2RTN,approachSim.targetAngularVeloXYZ) + [0 0 -omega];
    %-----------variables-----------------------%
    approachSim.chaserState = [initialChaserPose(:,j); initialChaserVelo(:,j)]; % hill's reference frame
    approachSim.keepOutHightMatrix = keepOutHightMatrix;
    approachSim.keepOutWidthMatrix = keepOutWidthMatrix;
    %-----------instanse------------------------%
    omega_t_max = vecnorm(approachSim.targetAngularVeloXYZ(1,:));
    omega_t_dot_max = vecnorm(target.InatiaMatrix\cross(targetAngularVelocity(1,:,j), target.InatiaMatrix*targetAngularVelocity(1,:,j)')');
    accelaretionConstraint = AccelaretionConstraint(uLim, vLim, attractivePose, omega_t_max, omega_t_dot_max);

    approachSim.aapf = aapf;
    approachSim.rvf = rvf;
    approachSim.accelaretionConstraint = accelaretionConstraint;
    approachSim.target = target;
    %-----------navigation error----------------%
    approachSim.enableNavigationError_position = enableNavigationError_position;
    approachSim.enableNavigationError_velocity = enableNavigationError_velocity;
    approachSim.enableNavigationError_attitude = enableNavigationError_attitude;
    approachSim.enableNavigationError_motion = enableNavigationError_motion;

    [~, pathRow, velocityRow, uRow, isArrived_, indexStampRow, rhoRow, rho_rRow] = approachSim.run();
    
    u(:,:,j) = uRow;
    velo(:,:,j) = velocityRow;
    path(:,:,j) = pathRow;
    isArrived(j) = isArrived_;
    indexStamp(:,j) = indexStampRow;
    rho(:,:,j) = rhoRow;
    rho_r(:,:,j) = rho_rRow;
    % toc
end
toc