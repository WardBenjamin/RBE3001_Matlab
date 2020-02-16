coms = initialize();

pid_config(coms, [.0007, .0004, 0], [.005 0 0.0001], [.005, 0, 0.001]);
pid_config(coms, [.0007, .0004, 0], [.005 0 0.0001], [.005, 0, 0.001]);

%% Set Up Timing
secondsToRecord = 5;
frequency = 5;
period = 1 / frequency;
loop_iterations = secondsToRecord * frequency;

%% Run Test Point 1

%% Define setpoints

arbitraryPosition1 = [250  80  20];
% arbitraryPosition1 = [200 -125 175];

testPoint1 = ikin(arbitraryPosition1)

setpoints = [setpoint(0, [0 0 0]), ...
    setpoint(2, testPoint1)];

    

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

model = stickModel(eye(4), eye(4), eye(4), []);

curr_setpoint = setpoints(1);

%% Collect data
tic
for idx = 1:loop_iterations %% Revise maximum to number of datapoints to be recorded
    current_time = toc;
    
    % Get the newest status packet
    returnPacket = status(coms);
    
    % Calculate forward kinematics
    [T, T1, T2, T3] = fwkin([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);
    
    % Log data to file
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
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
            [setpoint_idx, next_setpoint] = setpoint.getNextSetpoint(setpoints);
            
            if current_time >= next_setpoint.Time
                curr_setpoint = next_setpoint;
            end
            
            if setpoint_idx == length(setpoints) + 1
                % Stay at the last setpoint forever if we've run out of setpoints
                curr_setpoint = setpoints(end); 
            end
        end
        rad_setpoint = curr_setpoint.execute();
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


%% Close the csv file
fclose(csvfile);

%% Plot
figure(2);
grid on;

plot(times, joint1_values, 'r', times, joint2_values, 'g', times, joint3_values, 'b');
ylim([-pi, pi]);

xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Joint Angle vs Time: Inverse Kinematics Setpoint 1');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

set_setpoint(coms, [0 0 0]);

figure(3);
grid on;

plot(times, effX_pos, times, effY_pos, times, effZ_pos);
ylim([-350, 350]);

xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time: Inverse Kinematics Setpoint 1');
legend('X-Coordinate', 'Y-Coordinate', 'Z-Coordinate', 'Location', 'SouthWest');

% Clear up memory upon termination
coms.shutdown()