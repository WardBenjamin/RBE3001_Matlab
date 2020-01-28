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

secondsToRecord = 30;
frequency = 5;
period = 1 / frequency; % 50 Hz
loop_iterations = secondsToRecord * frequency;

%% Plot joint angles
joint1_values = 0;
joint2_values = 0;
joint3_values = 0;
time_values = 0;

hold on

plot(time_values, joint1_values, 'r', joint2_values, 'g', joint3_values, 'b');
ylim([-1100 1100]);
xlabel('Time (s)');
ylabel('Joint Angle (encoder tics)');
title('Joint PID Calibration');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

last_time = 0;

tic
for idx = 1:loop_iterations  
    current_time = toc;
    
    returnPacket = status(pp);
%     disp(returnPacket);
    
    joint1_values = [joint1_values, returnPacket(1)];
    joint2_values = [joint2_values, returnPacket(2)];
    joint3_values = [joint3_values, returnPacket(3)];
    time_values = [time_values, current_time];
    plot(time_values, joint1_values, 'r', joint2_values, 'g', joint3_values, 'b');
    drawnow;
    
    elapsed = toc;
    sleep_time = period - (elapsed - current_time)
    if sleep_time < 0
        sleep_time
        sleep_time = 0;
    end
    java.lang.Thread.sleep(sleep_time * 1000);
    
%     last_time = current_time
end


% Clear up memory upon termination
pp.shutdown()
