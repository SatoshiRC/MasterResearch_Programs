classdef target < environments
    %TARGET Summary of this class goes here
    % This class has target properties such as object specification and
    % trajectories. Also, this class has odefunction for position and
    % attitude calculation. its state vector refarences the Earth centered
    % equatorial coordinate system (ECS)
    
    properties
        Altitude
        Eccentricity
        Inclination
        RAAN
        ArgumentOfPerigee
        MeanAnomaly

        omega

        Ixx
        Iyy
        Izz

        %KOZ parameters
        b = [9 9 9]
        a = [2.5 2.5]      
    end
    
    methods
        function obj = target()
            obj@environments();
            %TARGET Construct an instance of this class
            %   Detailed explanation goes here
            obj.Altitude = 6978000;
            obj.Eccentricity = 0.0;
            % obj.Inclination = 98;
            obj.Inclination = 90;
            % obj.RAAN = 34.8;
            obj.RAAN = 0;
            obj.ArgumentOfPerigee =  0;
            obj.MeanAnomaly = 0;

            obj.Ixx = 27000;
            obj.Iyy = 27000;
            obj.Izz = 6500;
        end
        
        function [r, v] = getOrbitInIJK(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if obj.Eccentricity == 0
                if obj.Inclination == 0
                else
                    [r, v] = keplerian2ijk(obj.Altitude, obj.Eccentricity, obj.Inclination, obj.RAAN, obj.ArgumentOfPerigee, obj.MeanAnomaly, arglat=180);
                end
            else
                [r, v] = keplerian2ijk(obj.Altitude, obj.Eccentricity, obj.Inclination, obj.RAAN, obj.ArgumentOfPerigee, obj.MeanAnomaly);
            end
        end

        function state = initialState(obj)
            state = zeros([9, 1]);
            [state(4:6), state(1:3)] = obj.getOrbitInIJK();
        end

        function dState = EOM(obj, ~, state)
            %definition of state vector
            %   (1)-(3) velocity (x,y,z) in the ECS
            %   (4)-(6) position (x,y,z) in the ECS
            %   (7)-(9) force input (undefined)
            
            %variables
            position = state(4:6);
            positionNorm = norm(position);
            gravityAcceletion = obj.getGgravityCoffecient(position)/(positionNorm^2);

            % initialize return variable
            dState = zeros([9 1]);

            dState(1) = state(7) - state(4)/positionNorm * gravityAcceletion;
            dState(2) = state(8) - state(5)/positionNorm * gravityAcceletion;
            dState(3) = state(9) - state(6)/positionNorm * gravityAcceletion;
            dState(4) = state(1);
            dState(5) = state(2);
            dState(6) = state(3);
            dState(7) = 0;
            dState(8) = 0;
            dState(9) = 0;
        end

        function dState = EOM_rotation(obj, t, state, position, tt)
            %definition of state vector
            %   (1)-(3) angularVel (x,y,z) in body frame
            %   (4)-(7) attitude in quaternion from ECS to body frame
            %   (8)-(10) toque input in body frame
            % the quaternion represents point rotation.
            
            %variables
            position = interp1(tt,position,t).';
            positionNorm = sqrt(sumsqr(position));
            % calculate quaternion time derivertive from angular velosity
            omega = zeros([1 4]);
            omega(2:4) = state(1:3);
            q_omega = quaternion(omega);
            q = quaternion(transpose(state(4:7)));
            q_dot = 0.5*q*q_omega;
            q_dot = compact(q_dot);

            %gravity torque
            omega = state(1:3);
            rotationMat = rotmat(q,"frame");
            positionNormalizedVec = position/positionNorm;
            positionNormalizedBodyFrame = rotationMat*positionNormalizedVec;
            torque_g = 3*obj.getGgravityCoffecient()/(positionNorm^3)*(cross(positionNormalizedBodyFrame,obj.InatiaMatrix()*positionNormalizedBodyFrame));
            % initialize return variable
            dState = zeros([10 1]);

            dState(1:3) = - cross(omega,obj.InatiaMatrix()*omega);
            dState(1:3) = dState(1:3)+torque_g;
            dState(1) = dState(1)/obj.Ixx;
            dState(2) = dState(2)/obj.Iyy;
            dState(3) = dState(3)/obj.Izz;

            dState(4) = q_dot(1);
            dState(5) = q_dot(2);
            dState(6) = q_dot(3);
            dState(7) = q_dot(4);
            dState(8:10) = [0 0 0];
        end

        function res = r_cb(obj, chaserPose)
            % param chaserPose is given in body frame
            res = zeros([3 1]);
        
            b = obj.b;
            a = obj.a;
        
            poseVec = chaserPose;
            if norm(poseVec)>0
                % avoid 0 divede
                poseVecNorm = poseVec / norm(poseVec);
            end

            tmp = [norm(poseVecNorm(1:2)) poseVecNorm(3)];
            theta = acos(tmp(2));
            if tmp(1) < 0
                theta = -theta;
            end
            alpha = theta + pi/2;
            if alpha < 0
                alpha = alpha + 2*pi;
            end
        
            if 0 <= alpha && alpha < pi/2
                d = 2*a(1)*b(1)^2*cos(alpha) / (b(1)^2*cos(alpha)^2 + a(1)^2*sin(alpha)^2);
            elseif pi/2 <= alpha && alpha < pi
                d = -2*a(2)*b(2)^2*cos(alpha) / (b(2)^2*cos(alpha)^2 + a(2)^2*sin(alpha)^2);
            elseif pi <= alpha && alpha < 3*pi/2
                d = 2*a(2)*b(3) / sqrt(b(3)^2*cos(alpha)^2 + 4*a(2)^2*sin(alpha)^2);
            elseif 3*pi/2 <= alpha && alpha < 2*pi
                d = 2*a(1)*b(3) / sqrt(b(3)^2*cos(alpha)^2 + 4*a(1)^2*sin(alpha)^2);
            end
            if norm(poseVec) < d
                return;
            end
            res = poseVec - d*poseVecNorm;
        end

        function [koz, theta] = koz(obj, plotPlaneNorm, pafVec)
            %params, plotPlaneNorm and pafVec, are given same frame.
            %plotPlaneNorm is a 3d vector represents a normalize vector
            koz = zeros([500 3]);
            
            originalPafVec = [1 0 0];
            n = cross(pafVec,originalPafVec);
            if norm(n) ~= 0
                n = n/norm(n);
            else
                n = [1 0 0];
            end
            theta = acos(dot(pafVec/norm(pafVec),originalPafVec))/2;
            q_body2eci = quaternion(cos(theta),n(1)*sin(theta),n(2)*sin(theta),n(3)*sin(theta));
        
            plotPlaneNorm = plotPlaneNorm / norm(plotPlaneNorm);
            
            b = obj.b;
            a = obj.a;
            len = length(koz(:,1));
            alpha = pi/len;
            q = quaternion(cos(alpha),plotPlaneNorm(1)*sin(alpha),plotPlaneNorm(2)*sin(alpha),plotPlaneNorm(3)*sin(alpha));
            onPlaneVec = cross(plotPlaneNorm,pafVec);
            if onPlaneVec == zeros(size(onPlaneVec))
                onPlaneVec = [1 0 0];
            end
            for i=1:len
                
                poseVec = onPlaneVec;
                onPlaneVec = rotatepoint(q,onPlaneVec);
                if norm(poseVec)>0
                    % avoid 0 divede
                    poseVecNorm = poseVec / norm(poseVec);
                end
            
                theta = acos(poseVecNorm(1));
                if poseVecNorm(2) < 0
                    theta = -theta;
                end
                alpha = theta + pi/2;
                if alpha < 0
                    alpha = alpha + 2*pi;
                end
            
                if 0 <= alpha && alpha < pi/2
                    d = 2*a(1)*b(1)^2*cos(alpha) / (b(1)^2*cos(alpha)^2 + a(1)^2*sin(alpha)^2);
                elseif pi/2 <= alpha && alpha < pi
                    d = -2*a(2)*b(2)^2*cos(alpha) / (b(2)^2*cos(alpha)^2 + a(2)^2*sin(alpha)^2);
                elseif pi <= alpha && alpha < 3*pi/2
                    d = 2*a(2)*b(3) / sqrt(b(3)^2*cos(alpha)^2 + 4*a(2)^2*sin(alpha)^2);
                elseif 3*pi/2 <= alpha && alpha < 2*pi
                    d = 2*a(1)*b(3) / sqrt(b(3)^2*cos(alpha)^2 + 4*a(1)^2*sin(alpha)^2);
                end


                koz(i,:) = rotateframe(q_body2eci,d*poseVecNorm);
            end
            theta = acos(dot(pafVec,originalPafVec))/2;
        end


        function J = InatiaMatrix(obj)
            J = zeros([3 3]);
            J(1,1) = obj.Ixx;
            J(2,2) = obj.Iyy;
            J(3,3) = obj.Izz;
        end
    end
end

