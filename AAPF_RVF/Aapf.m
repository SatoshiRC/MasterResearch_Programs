classdef Aapf
    properties
        shapingMatrix
        rho
        rhoDot
        rho0
        rhoDot0
        rhoUpper
        rhoUnder
        Ka
        uLim
        vLim
        dt
    end

    methods
        function obj = Aapf(rho, rhoDot, rhoUpper, rhoUnder, Ka, uLim, vLim, dt)
            arguments
                rho(6,1)
                rhoDot(6,1)
                rhoUpper(6,1)
                rhoUnder(6,1)
                Ka(1,1)
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
            obj.Ka = Ka;
            obj.uLim = uLim;
            obj.vLim = vLim;
            obj.dt = dt;
        end

        function [shapingMatrix,obj] = update(obj, r_cf, r_cfDot, v0)
            arguments
                obj(1,1) Aapf
                r_cf(3,1)
                r_cfDot(3,1)
                v0(1,1) 
            end
            distance_ = vecnorm(r_cf);

            % update attractive potential field for transpose motion
            obj.rho= obj.rho+ obj.dt*obj.rhoDot;
            for n=1:length(obj.rho)
                if obj.rho(n) > obj.rhoUpper(n)
                    obj.rho(n) = obj.rhoUpper(n);
                elseif obj.rho(n) < obj.rhoUnder(n)
                    obj.rho(n) = obj.rhoUnder(n);
                end
            end
            R = [obj.rho(1) obj.rho(2) obj.rho(3);
                0 obj.rho(4) obj.rho(5);
                0 0 obj.rho(6)];
        
            k = [2*obj.rho(1)*r_cf(1)+obj.rho(2)*r_cf(2)+obj.rho(3)*r_cf(3) obj.rho(1)*r_cf(2) obj.rho(1)*r_cf(3) 0 0 0;
                obj.rho(2)*r_cf(1) obj.rho(1)*r_cf(1)+2*obj.rho(2)*r_cf(2)+obj.rho(3)*r_cf(3) obj.rho(2)*r_cf(3) 2*obj.rho(4)*r_cf(2)+obj.rho(5)*r_cf(3) obj.rho(4)*r_cf(3) 0;
                obj.rho(3)*r_cf(1) obj.rho(3)*r_cf(2) obj.rho(1)*r_cf(1)+obj.rho(2)*r_cf(2)+2*obj.rho(3)*r_cf(3) obj.rho(5)*r_cf(2) 2*obj.rho(5)*r_cf(3)+obj.rho(4)*r_cf(2) 2*obj.rho(6)*r_cf(3)];

            V0_dot = 0;
            e = -v0*r_cf(1:3)/distance_ + obj.Ka*(R'*R)*r_cf(1:3);
            tmp = pinv(k)*(v0*(r_cfDot/distance_ - r_cf(1:3)*r_cf(1:3).'*r_cfDot/(distance_^3)) ...
                +V0_dot*(r_cf(1:3)/distance_) - obj.Ka*(R'*R)*r_cfDot - 0.5*e);
        
            obj.rhoDot = tmp;
            shapingMatrix = R'*R;
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