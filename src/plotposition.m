clear
clear java
clear classes;
clf

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

secondsToRecord = 30;
frequency = 5;
period = 1 / frequency; % 50 Hz
loop_iterations = secondsToRecord * frequency;

%% Plot joint angles
joint1_values = 0;
joint2_values = 0;
joint3_values = 0;
time_values = 0;

csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');

hold on
grid on

plot(time_values, joint1_values, 'r', time_values, joint2_values, 'g', time_values, joint3_values, 'b');
ylim([-1100 1100]);
xlabel('Time (s)');
ylabel('Joint Angle (encoder tics)');
title('Joint PID Calibration');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

tic
for idx = 1:loop_iterations  
    current_time = toc;
    
    returnPacket = status(pp);
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', returnPacket(1:6));
%     disp(returnPacket);
    
    % Add the joint values to the range sets
    joint1_values = [joint1_values, returnPacket(1)];
    joint2_values = [joint2_values, returnPacket(2)];
    joint3_values = [joint3_values, returnPacket(3)];
    
    % Add the current time to the domain
    time_values = [time_values, current_time];
    
    % Plot all data
    plot(time_values, joint1_values, 'r', time_values, joint2_values, 'g', time_values, joint3_values, 'b');
    
    % This is unfortunate (since it's slow) but required (legend is randomly numbered otherwise)
    legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest'); 
    
    % Draw the plot on the current graph figure
    drawnow;
    
    % Calculate the remaining loop time to sleep for
    elapsed = toc;
    sleep_time = period - (elapsed - current_time)
    
    % If the loop iteration has run over (rare), don't sleep
    % Haha we're tired and this does the job!
    if sleep_time < 0
        sleep_time;
        sleep_time = 0;
    end
    
    % Sleep for the remaining loop time
    java.lang.Thread.sleep(sleep_time * 1000);
end

fclose(csvfile);

% Clear up memory upon termination
pp.shutdown()
