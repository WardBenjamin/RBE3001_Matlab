clear
clear java
clear classes;
clear all;
close all;

addpath('../lib'); % Ensure that lib folder is on filepath

vid = hex2dec('3742');
pid = hex2dec('0007');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);


%% Calibrate
statusPacket = status(pp);
pause(.1)
statusPacket = status(pp);
pause(.1)
calibration(pp, statusPacket);
pause(.1);
calibration(pp, statusPacket);

statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);

pid_config(pp, [.0007, .0004, 0], [.0009, .008, .1], [.005, .0001, 0])
pid_config(pp, [.0007, .0004, 0], [.0009, .008, .1], [.005, .0001, 0])

set_setpoint(pp, [0,0,0]);
set_setpoint(pp, [0,0,0]);

%% Define setpoints
setpoints = [setpoint(0, [0 0 0]), setpoint(3, [0 -1 .2]), setpoint(6, [0 -.4 -.5]), setpoint(9, [0 0 0])]; %TODO: Set to correct points
curr_setpoint = setpoints(1);

%% Set up timing
secondsToRecord = 15;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;

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

model = stickModelBasic(eye(4), eye(4), eye(4), []);

%% Collect data

tic

for idx = 1:loop_iterations  
    current_time = toc;
    
    % Get the newest status packet
    returnPacket = status(pp);
    
    % Calculate forward kinematics
    [T, T1, T2, ~] = fwkin([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);
    
    % Log data to file
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
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
        set_setpoint(pp, enc_setpoint);
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

set_setpoint(pp, [0 0 0]);

%% Close the csv file
fclose(csvfile);

%% Plot
figure(2);
grid on;

plot(times, joint1_values, 'r', times, joint2_values, 'g', times, joint3_values, 'b');
ylim([-pi, pi]);

xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Joint Angle vs Time: 2D Motion Planning');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

figure(3);
grid on;

plot(times, effX_pos, times, effZ_pos);
ylim([-50, 350]);

xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time: 2D Motion Planning');
legend('X-Coordinate', 'Z-Coordinate', 'Location', 'SouthWest');

figure(4);
grid on;

plot(times(1:end-1), diff(joint1_values)/period, times(1:end-1), diff(joint2_values)/period, times(1:end-1), diff(joint3_values));
ylim([-2.5, 2.5]);
xlabel('Time(s)');
ylabel('Joint velocities (rad/s)');
title('Joint Velocity vs Time: 2D Motion Planning');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

figure(5);
grid on;

plot(effX_pos, effZ_pos);
xlim([0, 350]);
ylim([-50, 250]);

hold on;

% TODO: Do we need to do fwkin for setpoints to get the xz-plane positions
% of each?
[s1T, ~, ~, ~] = fwkin(setpoints(1).Position);
[s2T, ~, ~, ~] = fwkin(setpoints(2).Position);
[s3T, ~, ~, ~] = fwkin(setpoints(3).Position);

plot(s1T(1,end), s1T(3,end), '-o', s2T(1,end), s2T(3,end), '-o', s3T(1,end), s3T(3,end), '-o'); %% TODO: Add correct setpoints

xlabel('X position (mm)');
ylabel('Z position (mm)');
title('X-Z Plane position: 2D Motion Planning');
legend('Actual position', 'Setpoint 1', 'Setpoint 2', 'Setpoint 3', 'Location', 'NorthWest');

% Clear up memory upon termination
pp.shutdown()