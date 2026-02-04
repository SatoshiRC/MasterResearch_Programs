classdef utility
    %UTILITY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function obj = utility()
        end
        
        %
        % Refer to "Angles-only relative orbit determination in low earth
        % orbit" for the definitions of e_r, e_n and e_t.
        %
        function [res] = getRotationMatrixIJK2RTN(obj_, positionInIJK, velocityInIJK)
            res = inv(getRotationMatrixRTN2IJK(obj_, positionInIJK, velocityInIJK));
        end

        function [res] = getRotationMatrixRTN2IJK(~, positionInIJK, velocityInIJK)
            e_r = positionInIJK / norm(positionInIJK);
            tmp = cross(positionInIJK, velocityInIJK);
            e_n = tmp / norm(tmp);
            e_t = cross(e_n,e_r);

            res = [e_r e_t e_n];
        end

        function [res] = getDotRotationMatrixIJK2RTN(~, positionInIJK, velocityInIJK)
            omega = norm(velocityInIJK) / norm(positionInIJK);

            dot_er = omega * velocityInIJK / norm(velocityInIJK);
            dot_et = -omega * positionInIJK / norm(positionInIJK);

            res = [dot_er dot_et zeros([3 1])].';
        end

        function res = normalize(~,x)
            res = x/norm(x);
        end

        function [position, velocity] = translateROE2PoseVeloInRTN(~,ROE, u, a, v)
            % this function translate relative orbital elements(ROE) to position
            % and velocity in hill reference frame (RTN frame). 
            % the ROE given as follows
            % ROE(1) : delta semi-major axsis
            % ROE(2) : delta argument of latitude
            % ROE(3) : e_x
            % ROE(4) : e_y
            % ROE(5) : i_x
            % ROE(6) : i_y
            % u : The target's argument of latitude
            % return position and velocity vectors in RTN frame.
            % Each return values are normalized by a or v.
            position = zeros([3,1]);
            velocity = zeros([3,1]);
            position(1) = ROE(1) - ROE(3)*cos(u) - ROE(4)*sin(u);
            position(2) = a*ROE(2) - 3*ROE(1)*ROE(2)/2 - 2*ROE(4)*cos(u) - 2*ROE(3)*sin(u);
            position(3) = -ROE(6)*cos(u)+ROE(5)*sin(u);
            velocity(1) = -ROE(4)*cos(u)+ROE(3)*sin(u);
            velocity(2) = -3*ROE(1)/2 + 2*(ROE(3)*cos(u)+ROE(4)*sin(u));
            velocity(3) = ROE(5)*cos(u)+ROE(6)*sin(u);
            velocity = velocity*v/a;
        end
    end
end

