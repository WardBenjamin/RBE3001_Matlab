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

%% Run status command 6 times and record data in a .csv file with the timestamp as a name
csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');
for k=1:100 %% Revise maximum to number of datapoints to be recorded
    returnPacket=status(pp);
    stickModel(-enc2rad(returnPacket(1)), -enc2rad(returnPacket(2)), -enc2rad(returnPacket(3)));
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', returnPacket(1:11));
    pause(.5);
end
fclose(csvfile);


statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);

%Encoder1: 9.7666666667	Encoder2: 18.8125	Encoder3: -10.2064393939


% Clear up memory upon termination
pp.shutdown()

