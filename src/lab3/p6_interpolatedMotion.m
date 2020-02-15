coms = initialize();

%% Define setpoints
setpoints = [setpoint(.95, [250  80  20]), setpoint(1.85, [200 -125 175]), setpoint(2.7, [175 0 -34])]; %TODO: Set to correct points

%% Set up timing
steps = 10;
secondsToRecord = 5;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;

%% Interpolate linearized points
returnPacket = status(coms);
[T, ~] = fwkin([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);

%TODO: Not sure if my interpolation logic is correct here

full_trajectory = [];


x0 = T(1, end);
y0 = T(2, end);
z0 = T(3, end);
t0 = 0;

for idx = 1:(length(setpoints))
    next_setpoint = setpoints(idx);
    partial_trajectory = [];
    
    x = linspace(x0, next_setpoint.Position(1), steps);
    y = linspace(y0, next_setpoint.Position(2), steps);
    z = linspace(z0, next_setpoint.Position(3), steps);
    t = linspace(t0, next_setpoint.Time, steps);
    
    for jdx = 1:steps
        partial_trajectory = [partial_trajectory, setpoint(t(jdx), [x(jdx), y(jdx), z(jdx)])];
    end
    
    % TODO: Maybe use fancy calculations to put this data in a preallocated matrix
    full_trajectory = horzcat(full_trajectory, partial_trajectory);

    x0 = next_setpoint.Position(1);
    y0 = next_setpoint.Position(2);
    z0 = next_setpoint.Position(3);
    t0 = next_setpoint.Time;
end

curr_setpoint = full_trajectory(1);

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

model = stickModelBasic(eye(4), eye(4), eye(4), []);

%% Collect data

tic
idx = 1;

while 1
% for idx = 1:loop_iterations  
    current_time = toc;
    
    if current_time > secondsToRecord
        break;
    end
    
    % Get the newest status packet
    returnPacket = status(coms);
    
    % Calculate forward kinematics
    [T, T1, T2, ~] = fwkin([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);
    
    % Log data to file
%     fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
    % Display stick model
    stickModelBasic(T, T1, T2, model);

    % Store current values in log matrices
    times(idx) = current_time;
    joint1_values(idx) = -enc2rad(returnPacket(1));
    joint2_8values(idx) = -enc2rad(returnPacket(2));
    joint3_values(idx) = -enc2rad(returnPacket(3));
    effX_pos(idx) = T(1, end);
    effY_pos(idx) = T(2, end);
    effZ_pos(idx) = T(3, end);
    
    % Setpoint handling
    % Execute the next setpoint if the current one has passed
    if curr_setpoint.HasExecuted
        [setpoint_idx, next_setpoint] = setpoint.getNextSetpoint(full_trajectory);

        if current_time >= curr_setpoint.Time
            curr_setpoint = next_setpoint;
        end

        if setpoint_idx == length(full_trajectory) + 1
            % Stay at the last setpoint forever if we've run out of setpoints
            curr_setpoint = full_trajectory(end); 
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

set_setpoint(coms, [0 0 0]);

%% Close the csv file
% fclose(csvfile);

%% Plot
figure(2);
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

figure(3);
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

figure(4);
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

figure(5);
grid on;
plot(times(2:end-1), x_acc, times(2:end-1), y_acc, times(2:end-1), z_acc);
%ylim([-2.5, 2.5]); %TODO: Set appropriate limit
xlabel('Time(s)');
ylabel('Accelerations (mm/s^2)');
title('Effector Acceleration vs Time');
legend('X-Acceleration', 'Y-Acceleration', 'Z-Acceleration', 'Location', 'SouthWest');

coms.shutdown();