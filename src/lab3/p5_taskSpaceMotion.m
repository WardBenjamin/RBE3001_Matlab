coms = initialize();

%% Define setpoints
setpoints = [setpoint(0.85, [250  80  20]), setpoint(1.85, [200 -125 175]), setpoint(3, [175 0 0])]; %TODO: Set to correct points
curr_setpoint = setpoints(1);

%% Set up timing
secondsToRecord = 3;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;

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
    [T, T1, T2, ~] = fwkin3001([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);
    
    % Log data to file
%     fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
    % Display stick model
    stickModelBasic(T, T1, T2, model);

    % Store current values in log matrices
    times(idx) = current_time;
    joint1_values(idx) = -enc2rad(returnPacket(1));
    joint2_values(idx) = -enc2rad(returnPacket(2));
    joint3_values(idx) = -enc2rad(returnPacket(3));
    effX_pos(idx) = T(1, end);
    effY_pos(idx) = T(2, end);
    effZ_pos(idx) = T(3, end);
    
    % Setpoint handling
    
    % Execute the next setpoint if the current one has passed
    if curr_setpoint.HasExecuted
        [setpoint_idx, next_setpoint] = setpoint.getNextSetpoint(setpoints);

        if current_time >= curr_setpoint.Time
            curr_setpoint = next_setpoint;
        end

        if setpoint_idx == length(setpoints) + 1
            % Stay at the last setpoint forever if we've run out of setpoints
            curr_setpoint = setpoints(end); 
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
view(45,28);
axis equal;
xlim([-50 300]), ylim([-250 250]), zlim([-100 400]);
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Task Space Path');

figure(3);
plot(times, effX_pos, times, effY_pos, times, effZ_pos);
grid on;
ylim([-350, 350]);
xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time');
legend('X-Coordinate', 'Y-Coordinate', 'Z-Coordinate', 'Location', 'SouthWest');


coms.shutdown();