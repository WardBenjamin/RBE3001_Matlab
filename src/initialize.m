function coms = initialize()
clear
clear java
clear classes;
close all;
%cla;
% clc;

addpath('../lib'); % Ensure that lib folder is on filepath
addpath('util');
addpath('commands');
addpath('subroutines');

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
coms = PacketProcessor(myHIDSimplePacketComs);

%% Calibrate
status(coms);
pause(.1);
statusPacket = status(coms);
pause(.1);
calibration(coms, statusPacket);
pause(.1);
calibration(coms, statusPacket);
status(coms);
status(coms);
status(coms);

% Default PID config
pid_config(coms, [.0007, .0004, 0], [.005 0 0.0001], [.005, 0, 0.001]);
pid_config(coms, [.0007, .0004, 0], [.005 0 0.0001], [.005, 0, 0.001]);

end