classdef target_KOZ1 < target
    %TARGET Summary of this class goes here
    % This class has target properties such as object specification and
    % trajectories. Also, this class has odefunction for position and
    % attitude calculation. its state vector refarences the Earth centered
    % equatorial coordinate system (ECS)
    
    properties
        %KOZ parameters
         
        KOZ_r
        KOZ_theta = 5*pi/180
    end
    
    methods
        function obj = target_KOZ1()
            %TARGET Construct an instance of this class
            obj@target();

            %A target is approcimate as rectangle
            targetHight = 4; %[m]
            targetWidth = 11; %[m]
        
            % A chaser is approximated as cuboid
            chaserHight = 0.81;
            chaserWidth = 3.7;
            chaserDepth = 1.2;
            obj.KOZ_r = (sqrt(chaserDepth^2 + chaserWidth^2 + chaserHight^2) + sqrt(targetWidth^2+targetHight^2))/2/(1-exp(-3)); 
        end

        function res = r_cb(obj, chaserPose)
            arguments
                obj 
                chaserPose(3,1) double {mustBeFinite}
            end
            % param chaserPose is given in body frame
            res = zeros([3 1]);
        
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

            d = obj.KOZ_r*(1-exp(-abs(theta)/(pi/24)));

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

                d = obj.KOZ_r*(1-exp(-abs(theta)/(pi/24)));

                koz(i,:) = rotateframe(q_body2eci,d*poseVecNorm);
            end
            theta = acos(dot(pafVec,originalPafVec))/2;
        end

        function [X, Y, Z] = KOZ3D(obj, pafVec)
            %plotPlaneNorm is a 3d vector represents a normalize vector
            originalPafVec = [0 0 1];

            X = zeros([36 50]);
            Y = zeros([36 50]);
            Z = zeros([36 50]);

            r = zeros([1 50]);

            phi = linspace(0,2*pi,36);
            theta = linspace(0,pi,50);
            for i=1:50
                r(i) = obj.KOZ_r*(1-exp(-abs(theta(i))/(pi/36)));
                X(:,i) = r(i)*sin(theta(i))*cos(phi);
                Y(:,i) = r(i)*sin(theta(i))*sin(phi);
                Z(:,i) = r(i)*cos(theta(i));
            end

            originalKOZ = [reshape(X, [1 1800]); reshape(Y, [1 1800]); reshape(Z, [1 1800])];
            pafVecNorm = pafVec / norm(pafVec);
            R = zeros(3);
            R(:,3) = pafVecNorm;
            R(:,2) = cross(pafVecNorm,originalPafVec);
            if norm(R(:,2)) == 0
                R(:,2) = [0 1 0];
            end
            R(:,1) = cross(R(:,2),pafVecNorm);

            KOZ = R*originalKOZ;

            X = reshape(KOZ(1,:),[36 50]);
            Y = reshape(KOZ(2,:),[36 50]);
            Z = reshape(KOZ(3,:),[36 50]);
        end
    end
end

