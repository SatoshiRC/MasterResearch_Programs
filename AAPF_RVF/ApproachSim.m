classdef ApproachSim
properties
    %-----------constant params-----------------%
    uLim
    vLim
    steps
    dt
    attractivePose
    orbitalAngularVelocity

    Ku

    enabledControlNoise = false

    enableNavigationError_position = false
    enableNavigationError_velocity = false
    enableNavigationError_attitude = false
    enableNavigationError_motion = false
    %-----------pre-calculated variables--------%
    quaternion_BODY2RTN
    quaternion_ECI2BODY
    quaternion_ECI2RTN

    targetAngularVeloXYZ
    targetAngularVeloRTN

    %-----------variables-----------------------%
    chaserState % hill's reference frame

    keepOutHightMatrix
    keepOutWidthMatrix

    isArrived = 0;
    phaseTransitionStamp
    %-----------instanse------------------------%
    aapf
    rvf
    accelaretionConstraint
    target
end

methods
    function obj = ApproachSim()
    end

    function [obj, path, velocity, controlInput, isArrived, indexStamp, rho, rho_r] = run(obj)
        isArrived = 0;
        path = zeros([3, obj.steps]);
        velocity = zeros([3, obj.steps]);
        controlInput = zeros([3, obj.steps-1]);
        rho = zeros([length(obj.aapf.rho), obj.steps]);
        rho_r = zeros([length(obj.rvf.rho), obj.steps]);
        path(:,1) = obj.chaserState(1:3);
        velocity(:,1) = obj.chaserState(4:6);

        omega_ref = 0;
        last_omega_ref = 0;
        indexStamp = zeros([3,1]);

        A_max = obj.accelaretionConstraint.maximumRadiusToSync();

        chaserPose = obj.chaserState(1:3);
        chaserVelo = obj.chaserState(4:6);

        for i=1:obj.steps-1
            targetAngularVeloXYZ_ = obj.targetAngularVeloXYZ(i,:)';
            targetAngularVeloRTN_ = obj.targetAngularVeloRTN(i,:)';
            targetAttitude_BODY2RTN = obj.quaternion_BODY2RTN(i);

%%
            chaserPoseXY = rotateframe(conj(targetAttitude_BODY2RTN), chaserPose')';
            chaserVeloXY = rotateframe(conj(targetAttitude_BODY2RTN), chaserVelo')' - cross(targetAngularVeloXYZ_, chaserPoseXY);
            rotationalVeloDiff = (cross(targetAngularVeloXYZ_, chaserPoseXY) - cross(targetAngularVeloXYZ_, obj.attractivePose));
            v_cf = rotateframe(conj(targetAttitude_BODY2RTN), chaserVelo')' - rotationalVeloDiff;
            r_cf = chaserPoseXY - obj.attractivePose;

            [r_cf, v_cf, targetAttitude_BODY2RTN, targetAngularVeloXYZ_] = obj.addNavigationError(r_cf, v_cf, targetAttitude_BODY2RTN, targetAngularVeloXYZ_);
            chaserPoseXY = r_cf + obj.attractivePose;
            chaserPose = rotateframe((targetAttitude_BODY2RTN), chaserPoseXY')';
            rotationalVeloDiff2 = (cross(targetAngularVeloXYZ_, chaserPoseXY) - cross(targetAngularVeloXYZ_, obj.attractivePose));
            chaserVelo = rotateframe(targetAttitude_BODY2RTN,(v_cf+rotationalVeloDiff2)')';
            chaserVeloXY = rotateframe(conj(targetAttitude_BODY2RTN), chaserVelo')' - cross(targetAngularVeloXYZ_, chaserPoseXY);
            targetAngularVeloRTN_ = rotateframe(targetAttitude_BODY2RTN, targetAngularVeloXYZ_')' - [0; 0; obj.orbitalAngularVelocity];

            r_cb = obj.target.r_cb(chaserPoseXY);
            distance_ = norm(r_cf(1:3));

            %check
            % chaserPoseXY2 = rotateframe(conj(targetAttitude_BODY2RTN), chaserPose')';
            % chaserVeloXY2 = rotateframe(conj(targetAttitude_BODY2RTN), chaserVelo')' - cross(targetAngularVelo_, chaserPoseXY2);
            % rotationalVeloDiff2 = (cross(targetAngularVelo_, chaserPoseXY2) - cross(targetAngularVelo_, obj.attractivePose));
            % v_cf2 = rotateframe(conj(targetAttitude_BODY2RTN), chaserVelo')' - rotationalVeloDiff2;
            % r_cf2 = chaserPoseXY - obj.attractivePose;
            % if not(obj.isapprox(v_cf2, v_cf) && obj.isapprox(r_cf2, r_cf) && obj.isapprox(chaserPoseXY2, chaserPoseXY) && obj.isapprox(chaserVeloXY2, chaserVeloXY))
            %     disp("error")
            % end
%%
            % check wheather the chase has been reached to the target
            % if reach destination, calculation will break.
            % if not reach destination, continue to calculation.
            if obj.isTerminal(r_cf, v_cf, targetAngularVeloXYZ_)
                isArrived = i;
                indexStamp(3) = i;
                break
            end

            % If the chaser investigate the KOZ, simulation will be terminated.
            if obj.isInvestigateKOZ(r_cb)
                isArrived = -i;
                break;
            end

            % update attractive potential field for transpose motion
            [attractiveShapingMatrix, obj.aapf] = obj.aapf.update(r_cf,v_cf, obj.vLim);
            
            %-------------------------------
            if vecnorm(chaserPoseXY) > A_max
                v_des = -obj.vLim*chaserPose/norm(chaserPose);
                circularVelo = 0;
                omega_ref = 0;
                last_omega_ref = 0;
            else
                if indexStamp(2) == 0
                    indexStamp(2) = i-1;
                end
                % updateRotatinalPotential
                % calculate omega_p
                InartialMat = obj.target.InatiaMatrix();
                omega_ref = obj.accelaretionConstraint.rvfReferenceConstraint(vecnorm(targetAngularVeloXYZ_), vecnorm(InartialMat \ cross(targetAngularVeloXYZ_, InartialMat*targetAngularVeloXYZ_)), chaserPoseXY, last_omega_ref, obj.dt);
                if omega_ref<0
                    omega_ref = 0;
                end
                % if omega_p > omega_p_max
                %     omega_p = omega_p_max;
                % end

                % poseVec = chaserPoseXY;
                % if norm(poseVec)>0
                %     % avoid 0 divede
                %     poseVecNorm = poseVec / norm(poseVec);
                % end
                % tmp = [norm(poseVecNorm(1:2)) poseVecNorm(3)];
                % theta = acos(tmp(2));
                omega_p_max_2 = 0.01;
                if omega_ref > omega_p_max_2
                    omega_ref = omega_p_max_2;
                end

                [rotationalMatrix, obj.rvf] = obj.rvf.update(chaserPoseXY, chaserVeloXY, obj.attractivePose, omega_ref);

                w = cross(chaserPoseXY, obj.attractivePose);
                v_des = -attractiveShapingMatrix*r_cf ...
                -exp(1-r_cb.'*obj.keepOutWidthMatrix*r_cb)*(obj.keepOutHightMatrix*r_cf-r_cf.'*obj.keepOutHightMatrix*r_cf*obj.keepOutWidthMatrix*r_cb) ...
                +cross(rotationalMatrix*w,chaserPoseXY);
                v_des = rotateframe(targetAttitude_BODY2RTN,v_des.').';
                circularVelo =  cross(targetAngularVeloRTN_, chaserPose);

                omega_ref_dot = (omega_ref - last_omega_ref)/obj.dt;
                last_omega_ref = omega_ref;
            end

            % controll input is calculated in Hill's reference frame.
            % chaserVelo is in Hill's reference frame
            u_ = obj.Ku*(v_des+circularVelo-chaserVelo);
            u_ = obj.constraintControlInput(u_);

            if obj.enabledControlNoise
                %TODO : configure noise parameters
            end
        
            [~,transferRes]=ode45(@(t,x)EOM(t,x,u_,obj.target.omega),[0 obj.dt],obj.chaserState);

            obj.chaserState = transferRes(end,:)';
            chaserPose = obj.chaserState(1:3);
            chaserVelo = obj.chaserState(4:6);
            path(1:3, i+1) = chaserPose(1:3);
            velocity(1:3, i+1) = chaserVelo(1:3);
            controlInput(:,i)=rotateframe(conj(targetAttitude_BODY2RTN),u_.');
            rho(:,i) = obj.aapf.rho;
            rho_r(:,i) = obj.rvf.rho;
        end
    end

    function constrainedInput = constraintControlInput(obj, input)
        arguments
            obj(1,1) ApproachSim
            input(3,1) double {mustBeFinite}
        end
        if vecnorm(input) > obj.uLim
            constrainedInput = input*obj.uLim/vecnorm(input);
        else
            constrainedInput = input;
        end
    end

    function  isTerminal = isTerminal(obj, r_cf, v_cf, omega_t)
        arguments
            obj
            r_cf(3,1) double {mustBeFinite}
            v_cf(3,1) double {mustBeFinite}
            omega_t(3,1) double {mustBeFinite}
        end
        isTerminal = false;
        T = obj.dt*10;
        r_e = r_cf+v_cf*T +cross(omega_t, r_cf-obj.attractivePose)*T;
        J1 = sqrt(r_e(1)^2+r_e(2)^2) <= 0.12 && abs(r_e(3)) < 0.08;
        J2 = sqrt(r_cf(1)^2+r_cf(2)^2) <= 0.12 && abs(r_cf(3)) < 0.08;
        % J1 = true;
        if J1 && J2 
            isTerminal = true;
        end
    end

    function isInvestigate = isInvestigateKOZ(~, r_cb)
        arguments
            ~
            r_cb(3,1) double {mustBeFinite}
        end
        isInvestigate = false;
        if r_cb == zeros(size(r_cb))
            isInvestigate = true;
        end
    end

    function [position, velocity, targetAttitude, targetAngularVelo] = addNavigationError(obj, position, velocity, targetAttitude, targetAngularVelo)
        arguments
            obj ApproachSim
            position(3,1) double {mustBeFinite}
            velocity(3,1) double {mustBeFinite}
            targetAttitude(1,1) quaternion
            targetAngularVelo(3,1) double {mustBeFinite} 
        end

        % Relative Position Error is calculated here
        % The magunitude of error is given as normal distribution whose mean is equal to zero and 3-sigma is 0.05[m]
        % The direction of the error is given randomly.
        if(obj.enableNavigationError_position)
            errorDistance = random("Normal", 0, 0.05/1.0);
            errorVec = random("Uniform",-1,1,3,1);
            position = position + errorDistance*errorVec/norm(errorVec);
        end

        % Relative Velocity Error
        % 3-sigma is 0.05 [m/s] for each axsis
        if(obj.enableNavigationError_velocity)
            errorNorm = random("Normal", 0, 0.05/1.0);
            errorVec = random("Uniform",-1,1,3,1);
            velocity = velocity + errorNorm*errorVec/norm(errorVec);
        end

        % Motion Estimation Error
        % 1-sigma is 0.015 deg/s for each axsis
        if(obj.enableNavigationError_motion)
            errorNorm = random("Normal", 0, 0.015*pi/180/1.0);
            errorVec = random("Uniform",-1,1,3,1);
            targetAngularVelo = targetAngularVelo + errorNorm*errorVec/norm(errorVec);
        end

        % Target Attitude Error
        % The magunitude of error is given as normal distribution whose mean is equal to zero and 3-sigma is 1[deg]
        if(obj.enableNavigationError_motion)
            errorAngle = random("Normal", 0, (1*pi/180)/1.0) / 2;
            errorVec = random("Uniform",-1,1,3,1);
            errorVec = errorVec / norm(errorVec);
            errorQuaternion = quaternion(cos(errorAngle), errorVec(1)*sin(errorAngle), errorVec(2)*sin(errorAngle), errorVec(3)*sin(errorAngle));
            targetAttitude = targetAttitude*errorQuaternion;
        end

    end

    function [logical] = isapprox(obj, A, B, tol)
        arguments
            obj(1,1)
            A(3,1)
            B(3,1)
            tol(1,1) = 1e-9
        end
        logical = true;
        for i=1:3
            if(not(isapprox(A(i), B(i), AbsoluteTolerance=tol)))
                logical = false;
                return;
            end
        end
        return;

    end
end

end