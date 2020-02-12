coms = initialize();

%% Define setpoints
setpoints = [setpoint(3, [0 -1 .2]), setpoint(6, [0 -.4 -.5]), setpoint(9, [0 0 0])]; %TODO: Set to correct points
curr_setpoint = setpoints(1);

%% Set up timing
steps = 10;
secondsToRecord = 15;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;

%% Interpolate linearized points
returnPacket = status(coms);
[T, T1, T2, ~] = fwkin3001([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);

%TODO: Not sure if my interpolation logic is correct here

fullTrajectory = zeros(length(setpoints) * steps, 1);
% Current Position - Setpoint 1
x0 = T(1, end);
y0 = T(2, end);
z0 = T(3, end);
t0 = 0; % Initial time by definition 0
curr_setpoint = curr_setpoint.getNextSetpoint(setpoints);
x1 = curr_setpoint.Position(1);
y1 = curr_setpoint.Position(2);
z1 = curr_setpoint.Position(3);
t1 = curr_setpoint.Time;
x = linspace(x0, x1, steps);
y = linspace(y0, y1, steps);
z = linspace(z0, z1, steps);
t = linspace(t0, t1, steps);
for i=1:steps
    fullTrajectory(i) = setpoint(t(i), [x(i),y(i),z(i)]);
end

% Setpoint 1 - 2
x0=x1; y0=y1; z0=z1; t0=t1;
curr_setpoint = curr_setpoint.getNextSetpoint(setpoints);
x1 = curr_setpoint.Position(1);
y1 = curr_setpoint.Position(2);
z1 = curr_setpoint.Position(3);
t1 = curr_setpoint.Time;
x = linspace(x0, x1, steps);
y = linspace(y0, y1, steps);
z = linspace(z0, z1, steps);
t = linspace(t0, t1, steps);
for i=1:steps
    fullTrajectory(i + steps) = setpoint(t(i), [x(i),y(i),z(i)]);
end

% Setpoint 2 - 3
x0=x1; y0=y1; z0=z1; t0=t1;
curr_setpoint = curr_setpoint.getNextSetpoint(setpoints);
x1 = curr_setpoint.Position(1);
y1 = curr_setpoint.Position(2);
z1 = curr_setpoint.Position(3);
t1 = curr_setpoint.Time;
x = linspace(x0, x1, steps);
y = linspace(y0, y1, steps);
z = linspace(z0, z1, steps);
t = linspace(t0, t1, steps);
for i=1:steps
    fullTrajectory(i + steps * 2) = setpoint(t(i), [x(i),y(i),z(i)]);
end

curr_setpoint = setpoints(1); % Reset current setpoint to first setpoint
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

model = stickModel(eye(4), eye(4), eye(4), []);

%% Collect data

tic

for idx = 1:loop_iterations  
    current_time = toc;
    
    % Get the newest status packet
    returnPacket = status(coms);
    
    % Calculate forward kinematics
    [T, T1, T2, ~] = fwkin3001([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);
    
    % Log data to file
%     fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
    % Display stick model
    stickModel(T, T1, T2, model);

    % Store current values in log matrices
    times(idx) = current_time;
    joint1_values(idx) = -enc2rad(returnPacket(1));
    joint2_values(idx) = -enc2rad(returnPacket(2));
    joint3_values(idx) = -enc2rad(returnPacket(3));
    effX_pos(idx) = T(1, end);
    effY_pos(idx) = T(2, end);
    effZ_pos(idx) = T(3, end);
    
    % Setpoint handling
    if current_time >= curr_setpoint.Time 
        % Execute the next setpoint if the current one has passed
        if curr_setpoint.HasExecuted 
            [setpoint_idx, next_setpoint] = setpoint.getNextSetpoint(fullTrajectory);
            
            if current_time >= next_setpoint.Time
                curr_setpoint = next_setpoint;
            end
            
            if setpoint_idx == length(fullTrajectory) + 1
                % Stay at the last setpoint forever if we've run out of setpoints
                curr_setpoint = setpoints(end); 
            end
        end
        rad_setpoint = ikin(curr_setpoint.execute());
        enc_setpoint = [-rad2enc(rad_setpoint(1)), -rad2enc(rad_setpoint(2)), -rad2enc(rad_setpoint(3))];
        set_setpoint(coms, enc_setpoint);
    end

    % Calculate the remaining loop time to sleep for
    elapsed = toc;
    sleep_time = period - (elapsed - current_time);
    
    % If the loop iteration has run over (rare), don't sleep
    % Haha we're tired and this does the job!
    if sleep_time < 0
        sleep_time;
        sleep_time = 0;
    end
    
    % Sleep for the remaining loop time
    java.lang.Thread.sleep(sleep_time * 1000);
end

set_setpoint(coms, [0 0 0]);

%% Close the csv file
% fclose(csvfile);

%% Plot
figure(2);
grid on;
plot3(effX_pos, effY_pos, effZ_pos);
view(45,28);
axis equal;
xlim([-50 300]), ylim([-250 250]), zlim([-100 400]);
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Task Space Path');

figure(3);
grid on;
plot(times, effX_pos, times, effY_pos, times, effZ_pos);
ylim([-50, 350]);
xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time');
legend('X-Coordinate', 'Y-Coordinate', 'Z-Coordinate', 'Location', 'SouthWest');

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
z_acc = diff(z_acc)/period;

figure(5);
grid on;
plot(times(2:end-1), x_acc, times(2:end-1), y_acc, times(2:end-1), z_acc);
%ylim([-2.5, 2.5]); %TODO: Set appropriate limit
xlabel('Time(s)');
ylabel('Accelerations (mm/s^2)');
title('Effector Acceleration vs Time');
legend('X-Acceleration', 'Y-Acceleration', 'Z-Acceleration', 'Location', 'SouthWest');

coms.shutdown();