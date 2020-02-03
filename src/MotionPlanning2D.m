clear
clear java
clear classes;

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
calibrate(pp, statusPacket);
puase(.1);
calibrate(pp, statusPacket);

%% Define setpoints
setpoints = [setpoint(0, [0 0 0]), setpoint(2, [0 0 0]), setpoint(4, [0 0 0])]; %TODO: Set to correct points
curr_setpoint = setpoints(1);

%% Set up timing
secondsToRecord = 30;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;

%% Set up data collection
csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');
times = zeroes(1, loop_iterations);
joint1_values = zeroes(1, loop_iterations);
joint2_values = zeroes(1, loop_iterations);
joint3_values = zeroes(1, loop_iterations);
effX_pos = zeroes(1, loop_iterations);
effY_pos = zeroes(1, loop_iterations);
effZ_pos = zeroes(1, loop_iterations);

%% Collect data

tic

for idx = 1:loop_iterations  
    current_time = toc;
    
    returnPacket = status(pp);
    
    returnPacket = status(pp);
    [T, T1, T2, T3] = fwkin3001(returnPacket(1:3));
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
%     disp(returnPacket);
    times(idx) = current_time;
    joint1_values(idx) = returnPacket(1);
    joint2_values(idx) = returnPacket(2);
    joint3_values(idx) = returnPacket(3);
    effX_pos = T(1,end);
    effY_pos = T(2, end);
    effZ_pos = T(3, end);
    
    if(curr_setpoint.Time > current_time) 
        curr_setpoint = getNextSetpoint(setpoints);
        set_setpoint(pp, curr_setpoint.position);

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
figure;
grid on;
plot(times, joint1_angle, 'r', times, joint2_angle, 'g', times, joint3_angle, 'b');
xlabel('Time (s)');
ylabel('Joint Angle (encoder tics)');
title('Joint Angle vs Time: 2D Motion Planning');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

figure;
grid on;
plot(times, effX_pos, times, effZ_pos);
xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time: 2D Motion Planning');
legend('X-Coordinate', 'Z-Coordinate', 'Location', 'SouthWest');

figure;
grid on;
plot(times, diff(joint1_angle)/period, times, diff(joint2_angle)/period, times, diff(joint3_angle));
xlabel('Time(s)');
ylabel('Joint velocities (m/s)'); %%TODO: Is this m/s?
title('Joint Velocity vs Time: 2D Motion Planning');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

figure;
grid on;
plot(effX_pos, effZ_Pos);
hold on;
plot(0, 0, '-o', 0, 0, '-o', 0, 0, '-o'); %% TODO: Add correct setpoints
legend('Actual position', 'Setpoint 1', 'Setpoint 2', 'Setpoint 3', 'Location', 'SouthWest');

% Clear up memory upon termination
pp.shutdown()