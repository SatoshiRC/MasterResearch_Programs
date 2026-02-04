classdef environments
    %ENVIRONMENTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        gravityCoffecient;
    end
    
    methods
        function obj = environments()
            %ENVIRONMENTS Construct an instance of this class
            %   Detailed explanation goes here
            obj.gravityCoffecient = 3.986004418 * 10^14;
        end
        
        function outputArg = getGgravityCoffecient(obj, position)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.gravityCoffecient;
        end
    end
end

