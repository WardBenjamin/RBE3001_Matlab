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

%% Set up data collection
csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');

%% Set up timing
secondsToRecord = 30;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;
tic

%% Collect data
for idx = 1:loop_iterations  
    current_time = toc;
    
    returnPacket = status(pp);
    
    returnPacket = status(pp);
    [T, T1, T2, T3] = fwkin3001(returnPacket(1:3));
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
%     disp(returnPacket);
    
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

%% Plot


%% Close the csv file
fclose(csvfile);

% Clear up memory upon termination
pp.shutdown()