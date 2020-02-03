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

statusPacket = status(pp);
pause(.1)
statusPacket = status(pp);
pause(.1)
calibrate(pp, statusPacket);
puase(.1);
calibrate(pp, statusPacket);

ai = [zeros(1, 3), zeros(1, 3), zeros(1, 3)];
tf = 5; %% Number of seconds
thetaf = [pi/4, pi/2, pi/2]; %% Angle setpoint in radians
steps = 30; %% Number of steps to take to reach trajectory

statusPacket = status(pp);
vi = statusPacket(4:6);
thetai = statusPacket(1:3);
for i = 1:3
    ai(i) = CuPolSolve(0, tf, vi(i), 0, thetai(i), thetaf(i));
end

% t_intervals = linspace(, tf,  steps + 1);
% 
% for i = 1:steps
    
%% Run status command 6 times and record data in a .csv file with the timestamp as a name
joint1_velocities = zeroes(1, steps);
joint2_velocities = zeroes(1, steps);
joint3_velocities = zeroes(1, steps);

joint1_values = zeroes(1, steps);
joint2_values = zeroes(1, steps);
joint3_values = zeroes(1, steps);

time_values = zeroes(1, steps);

csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');

tic
for k=1:steps %% Revise maximum to number of datapoints to be recorded
    current_time = toc;
    returnPacket=status(pp);
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', returnPacket(1:6));
    
    joint1_values(k) = returnPacket(1);
    joint2_values(k) = returnPacket(2);
    joint3_values(k) = returnPacket(3);
    
    joint1_velocities(k) = returnPacket(4);
    joint2_velocities(k) = returnPacket(5);
    joint3_velocities(k) = returnPacket(6);
    
    time_values(k) = current_time;
    
    set_setpoint(q(1, k), q(2, k), q(3, k));
    
    % Calculate the remaining loop time to sleep for
    elapsed = toc;
    sleep_time = t_intervals(k) - toc;
    
    % If the loop iteration has run over (rare), don't sleep
    % Haha we're tired and this does the job!
    if sleep_time < 0
        sleep_time;
        sleep_time = 0;
    end
    
    % Sleep for the remaining loop time
    java.lang.Thread.sleep(sleep_time * 1000);
end

hold on
grid on

f1 = figure1;
plot(time_values, joint1_values, 'r', time_values, joint2_values, 'g', time_values, joint3_values, 'b');
ylim([-1100 1100]);
xlabel('Time (s)');
ylabel('Joint Angle (encoder tics)');
title('Joint Angles');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

f2 = figure2;
plot(time_values, joint1_velocities, 'r', time_values, joint2_velocities, 'g', time_values, joint3_velocities, 'b');
ylim([-1100 1100]);
xlabel('Time (s)');
ylabel('Joint Velocity (m/s)');
title('Joint Velocities');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

fclose(csvfile);