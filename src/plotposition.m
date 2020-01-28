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

secondsToRecord = 5;
frequency = 40;
period = 1 / frequency; % 50 Hz
loop_iterations = secondsToRecord * frequency;

%% Plot Base joint angle
joint_values = 0;
time_values = 0;
for idx = 1:loop_iterations
    returnPacket = status(pp);
    joint_values = [joint_values, returnPacket(1)]
    elapsed = toc;
    time_values = [time_values, elapsed]
    sleep_time = period - (elapsed - 1)
    java.lang.Thread.sleep(sleep_time * 1000);
    toc
end
% plot 
% Clear up memory upon termination
pp.shutdown()
