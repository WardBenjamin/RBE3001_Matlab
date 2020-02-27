function [trajs, yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask] = genTrajectories(cam, q0, T_base_check, T_cam_check, cameraParams)
    COLOR_YELLOW = 1;
    COLOR_GREEN = 2;
    COLOR_BLUE = 3;
    
    %TODO: Fix x-coordinates for dropoff
    YELLOW_DROP = [[200 220 -20]; [200 220 -20]];
    GREEN_DROP = [[150 220 -20]; [150 220 -20]];
    BLUE_DROP = [[100 220 -20]; [100 220 -20]];
    
    %% Collect Image
    image = snapshot(cam);

    %% Obtain Object Locations
    [yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask] = findObjs(image, inv(T_base_check), T_cam_check, cameraParams);
    
    %% Identify Effector Setpoint
    
    color = 0;
    objs = [];
    if ~isempty(yellowObjs)
        objs = yellowObjs;
        color = COLOR_YELLOW;
    elseif ~isempty(greenObjs)
        objs = greenObjs;
        color = COLOR_GREEN;
    elseif ~isempty(blueObjs)
        objs = blueObjs;
        color = COLOR_BLUE;
    end
    
    if isempty(objs)
        trajs = [];
        return;
    end
    
    %% Compensate for weird transform effects
    % TODO: We don't actually know why the end point is offset by ~222mm
    
    for y = 1:length(objs(:,1))
        objs(y,2) = objs(y,2) + 222;
    end
    
    %% Generate main trajectory setpoints
    
    pickSetpoints = [setpoint(0.5, [q0(1), q0(2), 50]),...
        setpoint(1.5, [objs(1,1) + 20, objs(1,2), 50]),...
        setpoint(2.0, [objs(1,1) + 20, objs(1,2), 0])];
    
    if color == COLOR_YELLOW
        q1 = YELLOW_DROP(objs(1,3) + 1,:);
    elseif color == COLOR_BLUE
        q1 = BLUE_DROP(objs(1,3) + 1,:);
    else
        q1 = GREEN_DROP(objs(1,3) + 1,:);
    end
    
    placeSetpoints = [setpoint(2.5, [objs(1,1) + 20, objs(1,2), 50]),...
        setpoint(3.5, [q1(1), q1(2), 50]),...
        setpoint(4.0, q1)];



    %% Set Up Trajectory Plans
    trajs = [trajectory(pickSetpoints, q0), trajectory(placeSetpoints, [objs(1,1) + 20, objs(1,2), 0])];
end
