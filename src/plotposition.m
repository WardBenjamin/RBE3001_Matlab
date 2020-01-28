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

secondsToRecord = 10;
frequency = 10;
period = 1 / frequency; % 50 Hz
loop_iterations = secondsToRecord * frequency;

%% Plot Base joint angle
joint_values = 0;
time_values = 0;

hold on
plot(time_values, joint_values);
xlabel('Time (s)');
ylabel('Base Joint Angle (encoder tics)');
title('Base Joint PID Calibration');

last_time = 0;

tic
for idx = 1:loop_iterations  
    current_time = toc;
    
    returnPacket = status(pp);
    
    joint_values = [joint_values, returnPacket(1)];
    time_values = [time_values, current_time];
    
    plot(time_values, joint_values);
    drawnow;
    
    elapsed = toc;
    sleep_time = period - (elapsed - current_time)
    java.lang.Thread.sleep(sleep_time * 1000);
    
%     last_time = current_time
end



% Clear up memory upon termination
pp.shutdown()
