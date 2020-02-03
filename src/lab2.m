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

%% Send calibration packet
statusPacket = status(pp);
pause(.1);
statusPacket = status(pp);
pause(.1);
calibration(pp, statusPacket);
pause(.1);
calibration(pp, statusPacket);

model = stickModel(0, 0, 0, []);

%% Run status command 6 times and record data in a .csv file with the timestamp as a name
csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');
for k=1:10000 %% Revise maximum to number of datapoints to be recorded
    returnPacket=status(pp);
    stickModel(-enc2rad(returnPacket(1)), -enc2rad(returnPacket(2)), -enc2rad(returnPacket(3)), model);
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', returnPacket(1:11));
%     pause(.5);
end

setpoints = [setpoint(0, [0,0,0]) setpoint(2, [0,0,0])];

setpoints(1).execute()

tic
for idx = 1:loop_iterations  
    current_time = toc;
    
    nextPoint = setpoint.nextPoint(setpoints);
    if current_time >= nextPoint.Time
        position = nextPoint.execute();
        % Actually send the position here
    end
    
    returnPacket = status(pp);
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', returnPacket(1:6));

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

% Re-enable warnings for safety
warning('on', 'MATLAB:hg:DiceyTransformMatrix'); 
warning('on', 'MATLAB:gui:array:InvalidArrayClass');
