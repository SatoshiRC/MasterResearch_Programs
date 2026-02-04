classdef Rvf
    properties
        shapingMatrix
        rho
        rhoDot
        rho0
        rhoDot0
        rhoUpper
        rhoUnder
        Krotation
        uLim
        vLim
        dt

        omega_p = 0;
    end

    methods
        function obj = Rvf(rho, rhoDot, rhoUpper, rhoUnder, Krotation, uLim, vLim, dt)
            arguments
                rho(6,1)
                rhoDot(6,1)
                rhoUpper(6,1)
                rhoUnder(6,1)
                Krotation(1,1)
                uLim(1,1)
                vLim(1,1)
                dt(1,1)
            end
            obj.rho = rho;
            obj.rhoDot = rhoDot;
            obj.rho0 = rho;
            obj.rhoDot0 = rhoDot;
            obj.rhoUpper = rhoUpper;
            obj.rhoUnder = rhoUnder;
            obj.Krotation = Krotation;
            obj.uLim = uLim;
            obj.vLim = vLim;
            obj.dt = dt;
        end

        function [shapingMatrix,obj] = update(obj, r_ct, r_ctDot, attractivePose, omega_p)
            arguments
                obj(1,1) Rvf
                r_ct(3,1)
                r_ctDot(3,1)
                attractivePose(3,1)
                omega_p(1,1) double {mustBeReal, mustBeNonnegative, mustBeFinite}
            end

            omega_p_dot = (omega_p - obj.omega_p)/obj.dt;
            obj.omega_p = omega_p;

            obj.rho = obj.rho + obj.dt*obj.rhoDot;
            for n=1:length(obj.rho)
                if obj.rho(n) > obj.rhoUpper(n)
                    obj.rho(n) = obj.rhoUpper(n);
                elseif obj.rho(n) < obj.rhoUnder(n)
                    obj.rho(n) = obj.rhoUnder(n);
                end
            end
            R_r = [obj.rho(1) obj.rho(2) obj.rho(3);
                0 obj.rho(4) obj.rho(5);
                0 0 obj.rho(6)];

            w = cross(r_ct, attractivePose);
            w_dot = cross(r_ctDot, attractivePose);

            k = [2*obj.rho(1)*w(1)+obj.rho(2)*w(2)+obj.rho(3)*w(3) obj.rho(1)*w(2) obj.rho(1)*w(3) 0 0 0;
                obj.rho(2)*w(1) obj.rho(1)*w(1)+2*obj.rho(2)*w(2)+obj.rho(3)*w(3) obj.rho(2)*w(3) 2*obj.rho(4)*w(2)+obj.rho(5)*w(3) obj.rho(4)*w(3) 0;
                obj.rho(3)*w(1) obj.rho(3)*w(2) obj.rho(1)*w(1)+obj.rho(2)*w(2)+2*obj.rho(3)*w(3) obj.rho(5)*w(2) 2*obj.rho(5)*w(3)+obj.rho(4)*w(2) 2*obj.rho(6)*w(3)];

            e = omega_p*w/vecnorm(w) - obj.Krotation*(R_r'*R_r)*w;
            tmp = pinv(k)*(omega_p*(w_dot/vecnorm(w) - w*w.'*w_dot/(vecnorm(w)^3)) ...
                +omega_p_dot*(w/vecnorm(w)) - obj.Krotation*(R_r'*R_r)*w + 1*e);

            obj.rhoDot(1) = tmp(1);
            obj.rhoDot(2) = tmp(2);
            obj.rhoDot(3) = tmp(3);
            obj.rhoDot(4) = tmp(4);
            obj.rhoDot(5) = tmp(5);
            obj.rhoDot(6) = tmp(6);

            if isreal(obj.rhoDot) == false
                disp("error");
            end

            shapingMatrix = R_r.'*R_r;
        end

        function obj = reset(obj)
            obj.rho = obj.rho0;
            obj.rhoDot = obj.rhoDot0;
        end

        function obj = setParameter(obj, rho, rhoDot)
            arguments
                obj
                rho(6,1) double {mustBeFinite}
                rhoDot(6,1) double {mustBeFinite} = zeros([6,1])
            end
            obj.rho0 = rho;
            obj.rhoDot0 = rhoDot;
            obj.reset;
        end 
    end
end