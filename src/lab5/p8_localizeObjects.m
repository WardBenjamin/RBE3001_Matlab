clear
clear java
clear classes;
delete(findall(gcf,'type','annotation'));
close all;

%% Initialization and Timing
[coms, cam, cameraParams, T_base_check, T_cam_check] = initialize();
secondsToRecord = 3;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;
set_gripper(coms, 1); % Open gripper
set_gripper(coms, 1);

%% Generate Initial Trajectory
returnPacket = status(coms);
[T, ~] = fwkin(-enc2rad(returnPacket(1:3)));
q0 = T(1:3, end).';

DEBUG = true;

%% Capture initial image of workspace, detect objects, and generate initial trajectory.
% In debug mode, we display the image and object positions as a figure

if DEBUG
    [full_trajectory, yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask, sourceImage] = genTrajectories(cam, q0, T_base_check, T_cam_check, cameraParams);
    figure(2);
    imshow(sourceImage);
    hold on;
    if ~isempty(yellowObjs)
        viscircles(yellowObjs(:,4:5), yRadii, 'Color', 'y', 'LineWidth', 4);
    end
    if ~isempty(greenObjs)
        viscircles(greenObjs(:,4:5), gRadii, 'Color', 'g', 'LineWidth', 4);
    end
    if ~isempty(blueObjs)
        viscircles(blueObjs(:,4:5), bRadii, 'Color', 'b', 'LineWidth', 4);
    end
    figure(1);
else
    [full_trajectory, ~] = genTrajectories(cam, q0, T_base_check, T_cam_check, cameraParams);
end

if ~isempty(full_trajectory)
    curr_trajectory = full_trajectory(1);
    curr_setpoint = curr_trajectory.Setpoints(1);
end



%% Set up data collection
csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');

times = zeros(1, loop_iterations);

joint1_values = zeros(1, loop_iterations);
joint2_values = zeros(1, loop_iterations);
joint3_values = zeros(1, loop_iterations);

effX_pos = zeros(1, loop_iterations);
effY_pos = zeros(1, loop_iterations);
effZ_pos = zeros(1, loop_iterations);

%% Set up stick model visualization

figure(1);
model = stickModel(eye(4), eye(4), eye(4), [], zeros(3));
singularityThreshold = 8*10^6; %TODO: What should this be? Need to test.

%% Run main loop
% Execute trajectories and collect data

tic
idx = 1;
accTime = 0;
gripperStatus = 1;

while 1
    if isempty(full_trajectory)
        break;
    end
    current_time = toc;
          
    % Get the newest status packet
    returnPacket = status(coms);
    q = -enc2rad(returnPacket(1:3));
    q_dot = -enc2rad(returnPacket(4:6));
    
    % Calculate forward kinematics
    [T, T1, T2, ~] = fwkin(q);
    [p_dot, j0, jp, ~] = fwkindiff(q, q_dot);
    A = jp*jp.';
	manipEllipseVol = (4/3)*pi * sqrt(prod(eig(A)));
    
    % Display stick model
    stickModel(T, T1, T2, model, p_dot);
    
    if manipEllipseVol < singularityThreshold
        break;
    end

    % Store current values in log matrices
    times(idx) = accTime + current_time;
    joint1_values(idx) = q(1);
    joint2_values(idx) = q(2);
    joint3_values(idx) = q(3);
    effX_pos(idx) = T(1, end);
    effY_pos(idx) = T(2, end);
    effZ_pos(idx) = T(3, end);
    
    % Setpoint handling
    % Execute the next setpoint if the current one has passed
    if curr_setpoint.HasExecuted
        [setpoint_idx, next_setpoint] = curr_trajectory.getNextSetpoint();
        if current_time >= curr_setpoint.Time
            curr_setpoint = next_setpoint;
        end
        if curr_trajectory.HasFinished()
            % Toggle the gripper state
            gripperStatus = ~gripperStatus;
            
            java.lang.Thread.sleep(250); %% Attempting to give robot a chance to grip the orb thing
            set_gripper(coms, gripperStatus);
            set_gripper(coms, gripperStatus);
            java.lang.Thread.sleep(250); %% Attempting to give robot a chance to grip the orb thing
            
            accTime = accTime + toc;
            tic;
            
            [trajIdx, curr_trajectory] = curr_trajectory.getNextTrajectory(full_trajectory);
            if (length(full_trajectory) + 1) > trajIdx
                curr_setpoint = curr_trajectory.Setpoints(1);
            else
                % Capture image of workspace, detect objects, and generate new trajectory
                % In debug mode, we display the image and object positions as a figure
                if DEBUG
                    [full_trajectory, yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask, sourceImage] = genTrajectories(cam, T(1:3, end).', T_base_check, T_cam_check, cameraParams);
                    figure(2);
                    imshow(sourceImage);
                    hold on;
                    if ~isempty(yellowObjs)
                        viscircles(yellowObjs(:,4:5), yRadii, 'Color', 'y', 'LineWidth', 4);
                    end
                    if ~isempty(greenObjs)
                        viscircles(greenObjs(:,4:5), gRadii, 'Color', 'g', 'LineWidth', 4);
                    end
                    if ~isempty(blueObjs)
                        viscircles(blueObjs(:,4:5), bRadii, 'Color', 'b', 'LineWidth', 4);
                    end
                    figure(1);
                else
                    [full_trajectory, ~] = genTrajectories(cam, T(1:3, end).', T_base_check, T_cam_check, cameraParams);
                end
                
                if ~isempty(full_trajectory)
                    curr_trajectory = full_trajectory(1);
                    curr_setpoint = curr_trajectory.Setpoints(1);
                else
                    % If we have no more objects to pick up, we're done.
                    % Exit.
                    break;
                end
            end
        end
    else
        % Send the current setpoint from the trajectory
        rad_setpoint = ikin(curr_setpoint.execute());
        enc_setpoint = [-rad2enc(rad_setpoint(1)), -rad2enc(rad_setpoint(2)), -rad2enc(rad_setpoint(3))];
        set_setpoint(coms, enc_setpoint);
    end

    idx = idx + 1;
end

%% Reset to a home position
rad_setpoint = ikin([175 0 50]);
set_setpoint(coms, -rad2enc(rad_setpoint));

%% Close the csv file
fclose(csvfile);

%% Shut down communications
coms.shutdown();