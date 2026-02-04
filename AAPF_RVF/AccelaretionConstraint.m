classdef AccelaretionConstraint
properties
    accelaretionLim
    approachReferenceVelocity
    destination 
    omega_t_max
    omega_t_dot_max
end
methods
    function obj = AccelaretionConstraint(accelaretionLim, approachReferenceVelocity, destination, omega_t_max, omega_t_dot_max)
        arguments
            accelaretionLim(1,1) double
            approachReferenceVelocity(1,1) double
            destination(3,1) double
            omega_t_max(1,1) double
            omega_t_dot_max(1,1) double
        end
        obj.accelaretionLim = accelaretionLim;
        obj.approachReferenceVelocity = approachReferenceVelocity;
        obj.destination = destination;
        obj.omega_t_max = omega_t_max;
        obj.omega_t_dot_max = omega_t_dot_max;
    end

    function aMax = maximumRadiusToSync(obj)
        arguments
            obj
        end
        if (obj.omega_t_max^2+obj.omega_t_dot_max) < 10^-6
            aMax = inf;
            return;
        end
        aMax = (obj.accelaretionLim - 2*obj.omega_t_max*obj.approachReferenceVelocity)/(obj.omega_t_max^2+obj.omega_t_dot_max);
    end


    function rvfLimit = rvfReferenceConstraint(obj, targetAngularVelocity, targetAngularVelocityDotNorm, chaserPosition, last_omega_ref, dt)
        arguments
            obj
            targetAngularVelocity (1,1) double
            targetAngularVelocityDotNorm (1,1) double
            chaserPosition (3,1) double %%target body frame
            last_omega_ref (1,1) double {mustBeFinite}
            dt (1,1) double {mustBeFinite, mustBeNonnegative}
        end
        r_cf = vecnorm(chaserPosition - obj.destination);
        r_ct = vecnorm(chaserPosition);
        r_a = vecnorm(obj.destination);

        if r_cf <  1e-9
            rvfLimit = 0;
            return
        end
        rvfLimit = -(2*targetAngularVelocity*r_ct + 2*obj.approachReferenceVelocity*(1+r_a/r_cf) + r_ct/dt) ...
                + sqrt((2*targetAngularVelocity*r_ct + 2*obj.approachReferenceVelocity*(1+r_a/r_cf) + r_ct/dt)^2 ...
                -4*r_ct*(-obj.accelaretionLim + targetAngularVelocityDotNorm*r_ct - last_omega_ref*r_ct/dt + targetAngularVelocity^2*r_ct + 2*targetAngularVelocity*obj.approachReferenceVelocity));
        rvfLimit = rvfLimit / (2*r_ct);
    end
end

end