function stickModel = stickModel(theta_j1, theta_j2, theta_j3, lastModel)
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
    figure(1);
    
    hold on;
%     cla;
   
%     axis equal;  
  
    if isempty(lastModel)
        hp1 = plot3([p_0(1) p_1(1)], [p_0(2) p_1(2)], [p_0(3) p_1(3)], 'b', 'LineWidth', 2);
        hp2 = plot3([p_1(1) p_2(1)], [p_1(2) p_2(2)], [p_1(3) p_2(3)], 'g', 'LineWidth', 2);
        hp3 = plot3([p_2(1) p_3(1)], [p_2(2) p_3(2)], [p_2(3) p_3(3)], 'r', 'LineWidth', 2);
        grid on;
        view(45,28);
        xlim([-50 300]), ylim([-250 250]), zlim([-100 400]);
        xlabel('X [mm]');
        ylabel('Y [mm]');
        zlabel('Z [mm]');
        title('Stick Model of the RBE 3001 robot');
        stickModel = [triad('Scale', 30, 'LineWidth', 1, 'Matrix', T) triad('Scale', 30, 'LineWidth', 1, 'Matrix', T1) triad('Scale', 30, 'LineWidth', 1, 'Matrix', T1*T2), hp1, hp2, hp3];
        
        % Triads print a TON of warnings without turning these two types off
        warning('off', 'MATLAB:hg:DiceyTransformMatrix'); 
        warning('off', 'MATLAB:gui:array:InvalidArrayClass');
    else
        % TODO: Check if old handles are valid
%         if(ishandle(lastModel(4))...
        hp1 = lastModel(4);
        hp2 = lastModel(5);
        hp3 = lastModel(6);
        
        hp1.XData = [p_0(1) p_1(1)]; hp1.YData = [p_0(2) p_1(2)]; hp1.ZData = [p_0(3) p_1(3)];
        hp2.XData = [p_1(1) p_2(1)]; hp2.YData = [p_1(2) p_2(2)]; hp2.ZData = [p_1(3) p_2(3)];
        hp3.XData = [p_2(1) p_3(1)]; hp3.YData = [p_2(2) p_3(2)]; hp3.ZData = [p_2(3) p_3(3)];
        
        lastModel(1).Matrix = T;
        lastModel(2).Matrix = T1;
        lastModel(3).Matrix = T1*T2;

         % 	set(stickModel(1), 'Matrix', T);
%     set(stickModel(2), 'Matrix', T1);
%     set(stickModel(3), 'Matrix', T1*T2);
    

         %         plot3(stickModel(4), [p_0(1) p_1(1)], [p_0(2) p_1(2)], [p_0(3) p_1(3)], 'b', 'LineWidth', 2);
%         hp2 = plot3([p_1(1) p_2(1)], [p_1(2) p_2(2)], [p_1(3) p_2(3)], 'g', 'LineWidth', 2);
%         hp3 = plot3([p_2(1) p_3(1)], [p_2(2) p_3(2)], [p_2(3) p_3(3)], 'r', 'LineWidth', 2);
        stickModel = lastModel;
    end
    
%     hold off
end