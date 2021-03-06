coms = initialize();

loop_iterations = 1000;
times = zeros(loop_iterations, 1);

%% Collect data
for idx = 1:loop_iterations %% Revise maximum to number of datapoints to be recorded
    tic;

    coms.write(3, zeros(15, 1, 'single'));

    pause(0.003); % Minimum amount of time required between write and read
    coms.read(3);

    % Store current values in log matrices
    times(idx) = toc;
end

%% Plot
figure(1);

histogram(times);

% 
% grid on;
% 
% plot(times, joint1_values, 'r', times, joint2_values, 'g', times, joint3_values, 'b');
% ylim([-pi, pi]);
% 
% xlabel('Time (s)');
% ylabel('Joint Angle (rad)');
% title('Joint Angle vs Time: Inverse Kinematics Setpoint 1');
% legend('Joint 1', 'Joint 2', 'Joint 3', 'Location', 'SouthWest');
% 
% figure(3);
% grid on;
% 
% plot(times, effX_pos, times, effZ_pos);
% ylim([-50, 350]);
% 
% xlabel('Time (s)');
% ylabel('Effector position (mm)');
% title('Effector Position vs Time: Inverse Kinematics Setpoint 1');
% legend('X-Coordinate', 'Z-Coordinate', 'Location', 'SouthWest');


coms.shutdown();