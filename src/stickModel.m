function stickModel(theta_j1, theta_j2, theta_j3)
%     p_0 = [0, 0, 0]; % Position of joint 0, defined as origin
%     p_1 = [0, 0, 135]; % Position of joint 1
%     p_2 = getJ3Pos(theta_j1, theta_j2, theta_j3); % Position of joint 2
%     p_3 = fwkin3001(theta_j1, theta_j2, theta_j3); % Position of effector

    [T, T1, T2, T3] = fwkin3001(theta_j1, theta_j2, theta_j3);
    p_0 = [0 0 0];
    p_1 = T1(1:3,end);
    p_2 = T1*T2; p_2 = p_2(1:3,end);
    p_3 = T(1:3,end);
    
    % Plot each link on a 3D plot
    figure, hold on
    plot3([p_0(1) p_1(1)], [p_0(2) p_1(2)], [p_0(3) p_1(3)], 'b', 'LineWidth', 2);
    plot3([p_1(1) p_2(1)], [p_1(2) p_2(2)], [p_1(3) p_2(3)], 'g', 'LineWidth', 2);
    plot3([p_2(1) p_3(1)], [p_2(2) p_3(2)], [p_2(3) p_3(3)], 'r', 'LineWidth', 2);
    grid on, axis equal, hold off
    view(45,28);
    xlim([0 250]), ylim([-200 200]), zlim([-50 300]);
    xlabel('X [mm]'), ylabel('Y [mm]'), zlabel('Z [mm]');
    title('Stick Model of the RBE 3001 robot');
end