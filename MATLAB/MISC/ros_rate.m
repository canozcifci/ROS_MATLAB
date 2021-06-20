classdef ros_rate < matlab.System         % Public, tunable properties     
properties         
RATE;      
end

properties(DiscreteState)       

end

% Pre-computed constants     
properties(Access = private)         
    rateObj;     
end

methods(Access = protected)         
    
    function setupImpl(obj)             
        % Perform one-time calculations, such as computing constants             
        obj.rateObj = robotics.Rate(obj.RATE);         
    end
    function stepImpl(obj)             
        obj.rateObj.waitfor();           
    end
    function resetImpl(obj)        
    end
end
end