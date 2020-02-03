classdef setpoint < handle
    %SETPOINTS Position setpoint container
    %   Detailed explanation goes here
    
    properties
        Time                % Time to execute setpoint (s)
        Position = [0 0 0]  % Point to set in 3-space ([])
        HasExecuted = false % Whether this setpoint has been sent (boolean)
    end
    
    methods
        function obj = setpoint(time, position)
            obj.Time = time;
            obj.Position = position;
        end
        
        function pos = execute(obj)
            obj.HasExecuted = true;
            pos = obj.Position;
        end
    end
    
    methods(Static)
        function [n, nextPoint] = getNextSetpoint(setpoints)
            for n = 1:length(setpoints)
                if ~(setpoints(n).HasExecuted)
                    nextPoint = setpoints(n);
                    return
                end
            end
            n = length(setpoints) + 1;
            nextPoint = setpoints(end);
        end
    end
end

