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

% %% Collect Image
% image = snapshot(cam);
% 
% %% Obtain Object Locations
% [yellowObjs, greenObjs, blueObjs, blackObjs, yRadii, gRadii, bRadii, yMask, gMask, bMask, kMask] = findObjs(image, inv(T_base_check), T_cam_check, cameraParams);
% 
% figure(2);
% imshow(image);
% hold on;
% if ~isempty(yellowObjs)
%     viscircles(yellowObjs(:,4:5), yRadii, 'Color', 'y', 'LineWidth', 4);
% end
% if ~isempty(greenObjs)
%     viscircles(greenObjs(:,4:5), gRadii, 'Color', 'g', 'LineWidth', 4);
% end
% if ~isempty(blueObjs)
%     viscircles(blueObjs(:,4:5), bRadii, 'Color', 'b', 'LineWidth', 4);
% end

%% Display Object Data
% figure(2);
% imshow(yMask);
% 
% figure(3);
% imshow(gMask);
% 
% figure(4);
% imshow(bMask);
% 
% figure(5);
% imshow(kMask);

figure(3);
imshow(yMask | gMask | bMask);


%% Manually Fix Y-Transformation and Generate Trajectories
%TODO: Fix transformation so that y-coordinate is correct
%Added conditions to fix keep matrix index error on empty Objs matricies
%from occuring

yellTraj = [];
greenTraj = [];
blueTraj = [];

returnPacket = status(coms);
[T, ~] = fwkin(-enc2rad(returnPacket(1:3)));
initialPosition = T(1:3, end).';

if ~isempty(yellowObjs)

    yellTraj = genTrajectories(yellowObjs, 1, initialPosition);
end

if ~isempty(greenObjs)
    for g = 1:length(greenObjs(:,1))
        greenObjs(g,2) = greenObjs(g,2) + 222;
    end
    greenTraj = genTrajectories(greenObjs, 3, initialPosition);
end

if ~isempty(blueObjs)
    for b = 1:length(blueObjs(:,1))
        blueObjs(b,2) = blueObjs(b,2) + 222;
    end
    blueTraj = genTrajectories(blueObjs, 2, initialPosition);
end

%% Generate Trajectories
full_trajectory = trajectories(curr_trajectory).Setpoints;
[~, curr_setpoint] = trajectories(curr_trajectory).getNextSetpoint();

%% Set up data collection
% csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
% fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');

times = zeros(1, loop_iterations);

joint1_values = zeros(1, loop_iterations);
joint2_values = zeros(1, loop_iterations);
joint3_values = zeros(1, loop_iterations);

effX_pos = zeros(1, loop_iterations);
effY_pos = zeros(1, loop_iterations);
effZ_pos = zeros(1, loop_iterations);
figure(1);
model = stickModel(eye(4), eye(4), eye(4), [], zeros(3));
% singularityThreshold = 8*10^6; %TODO: What should this be? Need to test.
singularityThreshold = 8*10^6; %TODO: What should this be? Need to test.

%% Collect data

tic
idx = 1;
accTime = 0;

while 1
% for idx = 1:loop_iterations  
    current_time = toc;
    
    if current_time > secondsToRecord
        break;
    end
    
    % Get the newest status packet
    returnPacket = status(coms);
    q = -enc2rad(returnPacket(1:3));
    q_dot = -enc2rad(returnPacket(4:6));
    
    % Calculate forward kinematics
    [T, T1, T2, ~] = fwkin(q);
    [p_dot, j0, jp, ~] = fwkindiff(q, q_dot);
    A = jp*jp.';
	manipEllipseVol = (4/3)*pi * sqrt(prod(eig(A)));


    % Log data to file
%     fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
    % Display stick model
    stickModel(T, T1, T2, model, p_dot);
    
    if manipEllipseVol < singularityThreshold
        break;
    end

    % Store current values in log matrices
    times(idx) = current_time;
    joint1_values(idx) = q(1);
    joint2_values(idx) = q(2);
    joint3_values(idx) = q(3);
    effX_pos(idx) = T(1, end);
    effY_pos(idx) = T(2, end);
    effZ_pos(idx) = T(3, end);
    
    % Setpoint handling
    % Execute the next setpoint if the current one has passed
    if curr_setpoint.HasExecuted
        [setpoint_idx, next_setpoint] = trajectories(curr_trajectory).getNextSetpoint();

        if current_time >= curr_setpoint.Time
            curr_setpoint = next_setpoint;
        end

        if setpoint_idx == length(full_trajectory) + 1
            if length(trajectories) < curr_trajectory
                curr_trajectory = curr_trajectory + 1;
                curr_setpoint = trajectories(curr_trajectory).getNextSetpoint();
            else
                % Stay at the last setpoint forever if we've run out of setpoints
                curr_setpoint = full_trajectory(end); 
            end
        end
    else
        rad_setpoint = ikin(curr_setpoint.execute());
        enc_setpoint = [-rad2enc(rad_setpoint(1)), -rad2enc(rad_setpoint(2)), -rad2enc(rad_setpoint(3))];
        set_setpoint(coms, enc_setpoint);
    end
    
%     dist_temp = [effX_pos(idx), effY_pos(idx), effZ_pos(idx); curr_setpoint.Position];
%     pdist(dist_temp, 'euclidean')

    idx = idx + 1;
end

%close the gripper on object
%set_gripper(coms, 0);
% set_setpoint(coms, [0 0 0]);

%% Close the csv file
% fclose(csvfile);

%% Plot
figure(4);
grid on;
plot3(effX_pos, effY_pos, effZ_pos);
hold on;
plot3(250, 80, 20, 'o', 200, -125, 175, 'o', 175, 0, -34, 'o');
view(45,28);
axis equal;
xlim([-50 300]), ylim([-250 250]), zlim([-100 400]);
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Task Space Path');
legend('Path', 'Setpoint 1', 'Setpoint 2', 'Setpoint 3');

figure(5);
grid on;
plot(times, effX_pos, times, effY_pos, times, effZ_pos);
hold on;
plot([.95 1.85 2.7], [250 200 175], 'o', [.95 1.85 2.7], [80, -125, 0], 'o', [.95 1.85 2.7], [20, 175, -34], 'o'); 
ylim([-350, 350]);
xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time');
legend('X-Coordinate', 'Y-Coordinate', 'Z-Coordinate', 'X-Setpoints', 'Y-Setpoints', 'Z-Setpoints', 'Location', 'SouthWest');

x_vel = diff(effX_pos)/period;
y_vel = diff(effY_pos)/period;
z_vel = diff(effZ_pos)/period;

figure(6);
grid on;
plot(times(1:end-1), x_vel, times(1:end-1), y_vel, times(1:end-1), z_vel);
%ylim([-2.5, 2.5]); %TODO: Set appropriate limit
xlabel('Time(s)');
ylabel('Velocities (mm/s)');
title('Effector Velocity vs Time');
legend('X-Velocity', 'Y-Velocity', 'Z-Velocity', 'Location', 'SouthWest');

x_acc = diff(x_vel)/period;
y_acc = diff(y_vel)/period;
z_acc = diff(z_vel)/period;

figure(7);
grid on;
plot(times(2:end-1), x_acc, times(2:end-1), y_acc, times(2:end-1), z_acc);
%ylim([-2.5, 2.5]); %TODO: Set appropriate limit
xlabel('Time(s)');
ylabel('Accelerations (mm/s^2)');
title('Effector Acceleration vs Time');
legend('X-Acceleration', 'Y-Acceleration', 'Z-Acceleration', 'Location', 'SouthWest');

coms.shutdown();