function stickModel = stickModel(T, T1, T2, lastModel)

    % TODO: We can pass in fwkin in some places... potential speedup

%     T = transform(1); 
%     T1 = transform(2); 
%     T2 = transform(3);
    
    p_0 = [0 0 0];
    p_1 = T1(1:3,end);
    p_2 = T1*T2; p_2 = p_2(1:3,end);
    p_3 = T(1:3,end);
    
    % Plot each link on a 3D plot
    figure(1);
    
    hold on;
  
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

        stickModel = lastModel;
    end
end