clear
clear java
clear classes;
clear all;
close all;

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
calibration(pp, statusPacket);
pause(.1);
calibration(pp, statusPacket);
statusPacket = status(pp);
statusPacket = status(pp);
statusPacket = status(pp);

pid_config(pp, [.0007, .0004, 0], [.005 0 0.0001], [.005, 0, 0.001]);
pid_config(pp, [.0007, .0004, 0], [.005 0 0.0001], [.005, 0, 0.001]);

%% Define setpoints
setpoints = [setpoint(0, [0 0 0]), ...
    setpoint(3, [0 -1 .2]), setpoint(6, [0 -.4 -.5]), setpoint(9, [0 0 0])];

    
%% Set Up Timing
secondsToRecord = 15; %TODO: Define appropreiate length
steps = 30; %TODO: Define number of steps
% t_intervals = linspace(0, secondsToRecord, steps);
frequency = 5;
period = 1 / frequency;
loop_iterations = secondsToRecord * frequency;
    
%% Set Up Trajectory Plans
ai = zeros(3, (length(setpoints) - 1) * 4);
vi = 0;
vf = 0;

for s = 1:(length(setpoints) - 1) % Iterate through setpoints
    ti = setpoints(s).Time;
    tf = setpoints(s+1).Time;
    thetai=setpoints(s).Position;
    thetaf=setpoints(s+1).Position;
    for a = 1:3 % Iterate through axes
        cubic = CuPolSolve(ti, tf, vi, vf, thetai(a), thetaf(a))
        indices = sub2ind(size(ai), [s s s s], [(a-1)*4+1 (a-1)*4+2 (a-1)*4+3 (a-1)*4+4]);
        ai(indices) = cubic;
    end
end

%% Create Trajectory Setpoints

full_trajectory = setpoints(1);
q = zeros(1,3);

for s = 1:(length(setpoints) - 1)
    current_setpoint = setpoints(s);
    next_setpoint = setpoints(s+1);
    
    steps = 12;
    time_span = next_setpoint.Time - current_setpoint.Time;
    %% TODO: Can the robot actually process 5 setpoints per second? - Laks
    if time_span > 2
        % If the trajectory will generate less than 5 setpoints per second,
        % generate additional setpoints
        steps = time_span * (steps / 2);
    end
    
    for t = linspace(current_setpoint.Time, next_setpoint.Time, steps)
        for a = 1:3 % Iterate through axes
            q(a) = ai(s, (a-1)*4+1) + ai(s, (a-1)*4+2) * t + ai(s, (a-1)*4+3) * t^2 + ai(s, (a-1)*4+4) * t^3;
        end
        full_trajectory = [full_trajectory, setpoint(t, [q(1), q(2), q(3)])];
    end
end
    
%% Set up data collection
csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');

times = zeros(1, loop_iterations);

joint1_values = zeros(1, loop_iterations);
joint2_values = zeros(1, loop_iterations);
joint3_values = zeros(1, loop_iterations);

effX_pos = zeros(1, loop_iterations);
effY_pos = zeros(1, loop_iterations);
effZ_pos = zeros(1, loop_iterations);

model = stickModelBasic(eye(4), eye(4), eye(4), []);

curr_setpoint = full_trajectory(1);

%% Collect data
tic
for idx = 1:loop_iterations %% Revise maximum to number of datapoints to be recorded
    current_time = toc;
    
    % Get the newest status packet
    returnPacket = status(pp);
    
    % Calculate forward kinematics
    [T, T1, T2, T3] = fwkin([-enc2rad(returnPacket(1)) -enc2rad(returnPacket(2)) -enc2rad(returnPacket(3))]);
    
    % Log data to file
    fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
    % Display stick model
    stickModelBasic(T, T1, T2, model);

    % Store current values in log matrices
    times(idx) = current_time;
    joint1_values(idx) = -enc2rad(returnPacket(1));
    joint2_values(idx) = -enc2rad(returnPacket(2));
    joint3_values(idx) = -enc2rad(returnPacket(3));
    effX_pos(idx) = T(1, end);
    effY_pos(idx) = T(2, end);
    effZ_pos(idx) = T(3, end);
    
    % Setpoint handling
    if current_time >= curr_setpoint.Time 
        % Execute the next setpoint if the current one has passed
        if curr_setpoint.HasExecuted 
            [setpoint_idx, next_setpoint] = setpoint.getNextSetpoint(full_trajectory);
            
            if current_time >= next_setpoint.Time
                curr_setpoint = next_setpoint;
            end
            
            if setpoint_idx == length(full_trajectory) + 1
                % Stay at the last setpoint forever if we've run out of setpoints
                curr_setpoint = full_trajectory(end); 
            end
        end
        rad_setpoint = curr_setpoint.execute();
        enc_setpoint = [-rad2enc(rad_setpoint(1)), -rad2enc(rad_setpoint(2)), -rad2enc(rad_setpoint(3))];
        set_setpoint(pp, enc_setpoint);
    end

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


%% Close the csv file
fclose(csvfile);

%% Plot
figure(2);
grid on;

plot(times, joint1_values, 'r', times, joint2_values, 'g', times, joint3_values, 'b');
ylim([-pi, pi]);

xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Joint Angle vs Time: 2D Motion Planning');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

figure(3);
grid on;

plot(times, effX_pos, times, effZ_pos);
ylim([-50, 350]);

xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time: 2D Motion Planning');
legend('X-Coordinate', 'Z-Coordinate', 'Location', 'SouthWest');

figure(4);
grid on;

plot(times(1:end-1), diff(joint1_values)/period, times(1:end-1), diff(joint2_values)/period, times(1:end-1), diff(joint3_values));
ylim([-2.5, 2.5]);
xlabel('Time(s)');
ylabel('Joint velocities (rad/s)');
title('Joint Velocity vs Time: 2D Motion Planning');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');

figure(5);
grid on;

traj_x = [];
traj_z = [];

for s = 1:length(full_trajectory)
    sT = fwkin(full_trajectory(s).Position);
    traj_x = [traj_x, sT(1,end)];
    traj_z = [traj_z, sT(3,end)];
end

plot(traj_x, traj_z, '.g')

hold on;

plot(effX_pos, effZ_pos, 'b');
xlim([0, 350]);
ylim([-50, 250]);

setpoint_x = [];
setpoint_z = [];

for s = 1:length(setpoints)
    sT = fwkin(setpoints(s).Position);
    setpoint_x = [setpoint_x, sT(1,end)];
    setpoint_z = [setpoint_z, sT(3,end)];
end

plot(setpoint_x, setpoint_z, 'or')

xlabel('X position (mm)');
ylabel('Z position (mm)');
title('X-Z Plane position: 2D Motion Planning');
legend('Trajectory points', 'Setpoints', 'Actual position', 'Location', 'NorthWest');

% Clear up memory upon termination
pp.shutdown()