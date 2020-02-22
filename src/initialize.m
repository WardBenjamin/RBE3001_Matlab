function [coms, cameraParams, T_base_check] = initialize()
clear
clear java
% clear classes;
delete(findall(gcf,'type','annotation'));
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

cameraParams = load('cameraParams2.mat');
cameraParams = cameraParams.cameraParams2;
T_base_check = [-1,0,0,275.800000000000;0,1,0,113.600000000000;0,0,-1,0;0,0,0,1];

% Create a PacketProcessor object to send data to the nucleo firmware
coms = PacketProcessor(myHIDSimplePacketComs);

%% Calibrate
% status(coms);
% pause(.1);
% statusPacket = status(coms);
% pause(.1);
% calibration(coms, statusPacket);
% pause(.1);
% calibration(coms, statusPacket);
status(coms);
status(coms);
status(coms);

% Default PID config
pid_config(coms, [.0007, .0004, 0], [.003 0 0.00005], [.005, 0, 0.001]);
pid_config(coms, [.0007, .0004, 0], [.0012 0 0.00005], [.0015, 0, 0.001]);

set_setpoint(coms, [0 0 0]);
set_setpoint(coms, [0 0 0]);

% java.lang.Thread.sleep(250);
pause(.5);

status(coms);
status(coms);

end