function [trajs, yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask, sourceImage] = genTrajectories(cam, q0, T_base_check, T_cam_check, cameraParams)
% genTrajectories Calculates the quintic trajectory between the current end
% effector position and that of the next object to pick up. Detects objects
% using a webcam snapshot and findObjs.
%
%   Authors
%   -------
%   Benjamin Ward      <blward@wpi.edu>
%   Teresa Saddler     <tsaddler@wpi.edu>
%
%   Latest Revision
%   ---------------
%   03/04/2020

    COLOR_YELLOW = 1;
    COLOR_GREEN = 2;
    COLOR_BLUE = 3;
    
    YELLOW_DROP = [[200 220 -20]; [200 -220 -20]];
    GREEN_DROP = [[150 220 -20]; [150 -220 -20]];
    BLUE_DROP = [[100 220 -20]; [100 -220 -20]];
    
    %% Collect Image
    sourceImage = snapshot(cam);

    %% Obtain Object Locations
    [yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask] = findObjs(sourceImage, inv(T_base_check), T_cam_check, cameraParams);
    
    %% Identify Effector Setpoint
    
    color = 0;
    objs = [];
    if ~isempty(yellowObjs) && anyInside(yellowObjs)
        objs = yellowObjs;
        color = COLOR_YELLOW
    elseif ~isempty(greenObjs) && anyInside(greenObjs)
        objs = greenObjs;
        color = COLOR_GREEN
    elseif ~isempty(blueObjs) && anyInside(blueObjs)
        objs = blueObjs;
        color = COLOR_BLUE
    end
    
    if isempty(objs)
        trajs = [];
        return;
    end
    
    %% Compensate for weird transform effects
    % TODO: We don't actually know why the end point is offset by ~222mm
    
	objPosition = [];
    vOffset = 15;

    for y = 1:length(objs(:,1))
        if abs(objs(y,2)) < 175
            objPosition = objs(y, 1:2);
            objV = objPosition / norm(objPosition);
            objPosition = objPosition + objV * vOffset;
            objPosition(3) = objs(y,3);
            break;
        end
    end
    
	if isempty(objPosition)
        trajs = [];
        return;
    end
    
    %% Generate main trajectory setpoints
    
    hoverY = 100;
    grabY = -20;
    
    pickSetpoints = [setpoint(0.5, [q0(1), q0(2), hoverY]),...
        setpoint(1.5, [objPosition(1,1), objPosition(1,2), hoverY]),...
        setpoint(2.5, [objPosition(1,1), objPosition(1,2), grabY])];
    
    disp(objPosition(1,3) + 1);
    
    if color == COLOR_YELLOW
        q1 = YELLOW_DROP(objPosition(1,3) + 1,:);
    elseif color == COLOR_BLUE
        q1 = BLUE_DROP(objPosition(1,3) + 1,:);
    else
        q1 = GREEN_DROP(objPosition(1,3) + 1,:);
    end
    
    placeSetpoints = [setpoint(0.5, [objPosition(1,1), objPosition(1,2), hoverY]),...
        setpoint(3.5, [q1(1), q1(2), hoverY]),...
        setpoint(4.5, q1)];



    %% Set Up Trajectory Plans
    trajs = [trajectory(pickSetpoints, q0), trajectory(placeSetpoints, [objPosition(1,1), objPosition(1,2), grabY])];
end

function anyInside = anyInside(points)
    for idx = 1:length(points(:,1))
        if abs(points(idx, 2)) < 175
            disp(points(idx, 2));
            anyInside = true
            return;
        end
        anyInside = false
    end
end
