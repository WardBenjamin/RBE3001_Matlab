classdef trajectory < handle
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Setpoints = []      % Time to execute setpoint (s)
    end
    
    methods
        function obj = trajectory(setpoints, initial_position)
            if nargin > 0
                steps = 10;

                ai = zeros(3, length(setpoints) * 6);
                v0 = 0;
                vf = 0;
                a0 = 0;
                af = 0;

                t0 = 0;
                q0 = initial_position;
    
                %% Calculate quintic polynomial coefficients
                for s = 1:length(setpoints) % Iterate through setpoints
                    tf = setpoints(s).Time;
                    qf = setpoints(s).Position;

                    for a = 1:3 % Iterate through axes
                        quintic = quinPolSolve(t0, tf, a0, af, v0, vf, q0(a), qf(a));
                        indices = sub2ind(size(ai), [s s s s s s], [(a-1)*6+1 (a-1)*6+2 (a-1)*6+3 (a-1)*6+4 (a-1)*6+5 (a-1)*6+6]);
                        ai(indices) = quintic;
                    end

                    t0 = tf;
                    q0 = qf;
                end

                %% Create Trajectory Setpoints

                full_trajectory = [];
                q = zeros(1,3);

                current_setpoint = setpoint(0, q0);

                for s = 1:length(setpoints)
                    next_setpoint = setpoints(s);

                    for t = linspace(current_setpoint.Time, next_setpoint.Time, steps)
                        for a = 1:3 % Iterate through axes
                            q(a) = ai(s, (a-1)*6+1) + ai(s, (a-1)*6+2) * t + ai(s, (a-1)*6+3) * t^2 + ai(s, (a-1)*6+4) * t^3 + ai(s, (a-1)*6+5) * t^4+ ai(s, (a-1)*6+6) * t^5;
                        end
                        full_trajectory = [full_trajectory, setpoint(t, [q(1), q(2), q(3)])];
                    end

                    current_setpoint = next_setpoint;
                end
                
                obj.Setpoints = full_trajectory;
            end
        end
        
        function [n, nextPoint] = getNextSetpoint(obj)
            for n = 1:length(obj.Setpoints)
                if ~(obj.Setpoints(n).HasExecuted)
                    nextPoint = obj.Setpoints(n);
                    return
                end
            end
            n = length(obj.Setpoints) + 1;
            nextPoint = obj.Setpoints(end);
        end
        
        function hasFinished = HasFinished(obj)
            hasFinished = obj.Setpoints(end).HasExecuted;
        end
    end
end

