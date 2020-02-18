coms = initialize();

%% Define setpoints
setpoints = [setpoint(0.85, [250  80  20]), setpoint(1.75, [200 -125 175]), setpoint(2.6, [175 0 -34])]; %TODO: Set to correct points

%% Set up timing
steps = 10;
secondsToRecord = 5;
frequency = 5;
period = 1 / frequency; 
loop_iterations = secondsToRecord * frequency;

%% Set Up Trajectory Plans
returnPacket = status(coms);
[T, ~] = fwkin(-enc2rad(returnPacket(1:3)));

ai = zeros(3, length(setpoints) * 6);
v0 = 0;
vf = 0;
a0 = 0;
af = 0;

t0 = 0;
q0 = T(1:3, end).';
    
for s = 1:length(setpoints) % Iterate through setpoints
    tf = setpoints(s).Time;
    qf = setpoints(s).Position;

    for a = 1:3 % Iterate through axes
        quintic = quinPolSolve(t0, tf, a0, af, v0, vf, q0(a), qf(a))
        indices = sub2ind(size(ai), [s s s s s s], [(a-1)*6+1 (a-1)*6+2 (a-1)*6+3 (a-1)*6+4 (a-1)*6+5 (a-1)*6+6]);
        ai(indices) = quintic;
    end
    
    t0 = tf;
    q0 = qf;
end

%% Create Trajectory Setpoints

full_trajectory = [];
q = zeros(1,3);

current_setpoint = setpoint(0, T(1:3, end).');

for s = 1:length(setpoints)
    next_setpoint = setpoints(s);
    
    for t = linspace(current_setpoint.Time, next_setpoint.Time, steps)
        for a = 1:3 % Iterate through axes
            q(a) = ai(s, (a-1)*6+1) + ai(s, (a-1)*6+2) * t + ai(s, (a-1)*6+3) * t^2 + ai(s, (a-1)*6+4) * t^3 + ai(s, (a-1)*6+5) * t^4+ ai(s, (a-1)*6+6) * t^5;
        end
        full_trajectory = [full_trajectory, setpoint(t, [q(1), q(2), q(3)])];
    end
    
    current_setpoint = next_setpoint;
end

curr_setpoint = full_trajectory(1);

%% Set up data collection
% csvfile = fopen(sprintf('../logs/log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
% fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');

times = zeros(1, loop_iterations);

joint1_values = zeros(1, loop_iterations);
joint2_values = zeros(1, loop_iterations);
joint3_values = zeros(1, loop_iterations);

effX_pos = zeros(1, loop_iterations);
effY_pos = zeros(1, loop_iterations);
effZ_pos = zeros(1, loop_iterations);
model = stickModel(eye(4), eye(4), eye(4), [], zeros(3), eye(3), 0, 0);
singularityThreshold = 8*10^6; %TODO: What should this be? Need to test.

%% Collect data

tic
idx = 1;

while 1
% for idx = 1:loop_iterations  
    current_time = toc;
    
    if current_time > secondsToRecord
        break;
    end
    
    % Get the newest status packet
    returnPacket = status(coms);
    q = -enc2rad(returnPacket(1:3));
    q_dot = -enc2rad(returnPacket(4:6));
    
    % Calculate forward kinematics
    [T, T1, T2, ~] = fwkin(q);
    [p_dot, j0, jp, ~] = fwkindiff(q, q_dot);
    A = jp*jp.';
	manipEllipseVol = (4/3)*pi * sqrt(prod(eig(A)));


    % Log data to file
%     fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', current_time,returnPacket(1:3),T(1:3,end));
    
    % Display stick model
    stickModel(T, T1, T2, model, p_dot, A, manipEllipseVol, singularityThreshold);
    
    if manipEllipseVol < singularityThreshold
        break;
    end

    % Store current values in log matrices
    times(idx) = current_time;
    joint1_values(idx) = q(1);
    joint2_values(idx) = q(2);
    joint3_values(idx) = q(3);
    effX_pos(idx) = T(1, end);
    effY_pos(idx) = T(2, end);
    effZ_pos(idx) = T(3, end);
    
    % Setpoint handling
    % Execute the next setpoint if the current one has passed
    if curr_setpoint.HasExecuted
        [setpoint_idx, next_setpoint] = setpoint.getNextSetpoint(full_trajectory);

        if current_time >= curr_setpoint.Time
            curr_setpoint = next_setpoint;
        end

        if setpoint_idx == length(full_trajectory) + 1
            % Stay at the last setpoint forever if we've run out of setpoints
            curr_setpoint = full_trajectory(end); 
        end
    else
        rad_setpoint = ikin(curr_setpoint.execute());
        enc_setpoint = [-rad2enc(rad_setpoint(1)), -rad2enc(rad_setpoint(2)), -rad2enc(rad_setpoint(3))];
        set_setpoint(coms, enc_setpoint);
    end
    
%     dist_temp = [effX_pos(idx), effY_pos(idx), effZ_pos(idx); curr_setpoint.Position];
%     pdist(dist_temp, 'euclidean')

    idx = idx + 1;
end

set_setpoint(coms, [0 0 0]);

%% Close the csv file
% fclose(csvfile);

%% Plot
figure(2);
grid on;
plot3(effX_pos, effY_pos, effZ_pos);
hold on;
plot3(250, 80, 20, 'o', 200, -125, 175, 'o', 175, 0, -34, 'o');
view(45,28);
axis equal;
xlim([-50 300]), ylim([-250 250]), zlim([-100 400]);
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Task Space Path');
legend('Path', 'Setpoint 1', 'Setpoint 2', 'Setpoint 3');

figure(3);
grid on;
plot(times, effX_pos, times, effY_pos, times, effZ_pos);
hold on;
plot([.95 1.85 2.7], [250 200 175], 'o', [.95 1.85 2.7], [80, -125, 0], 'o', [.95 1.85 2.7], [20, 175, -34], 'o'); 
ylim([-350, 350]);
xlabel('Time (s)');
ylabel('Effector position (mm)');
title('Effector Position vs Time');
legend('X-Coordinate', 'Y-Coordinate', 'Z-Coordinate', 'X-Setpoints', 'Y-Setpoints', 'Z-Setpoints', 'Location', 'SouthWest');

x_vel = diff(effX_pos)/period;
y_vel = diff(effY_pos)/period;
z_vel = diff(effZ_pos)/period;

figure(4);
grid on;
plot(times(1:end-1), x_vel, times(1:end-1), y_vel, times(1:end-1), z_vel);
%ylim([-2.5, 2.5]); %TODO: Set appropriate limit
xlabel('Time(s)');
ylabel('Velocities (mm/s)');
title('Effector Velocity vs Time');
legend('X-Velocity', 'Y-Velocity', 'Z-Velocity', 'Location', 'SouthWest');

x_acc = diff(x_vel)/period;
y_acc = diff(y_vel)/period;
z_acc = diff(z_vel)/period;

figure(5);
grid on;
plot(times(2:end-1), x_acc, times(2:end-1), y_acc, times(2:end-1), z_acc);
%ylim([-2.5, 2.5]); %TODO: Set appropriate limit
xlabel('Time(s)');
ylabel('Accelerations (mm/s^2)');
title('Effector Acceleration vs Time');
legend('X-Acceleration', 'Y-Acceleration', 'Z-Acceleration', 'Location', 'SouthWest');

coms.shutdown();