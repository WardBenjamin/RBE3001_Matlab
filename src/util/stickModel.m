function stickModel = stickModel(T, T1, T2, lastModel, p_dot, A, manipEllipseVol, singularityThreshold)

    % TODO: We can pass in fwkin in some places... potential speedup

%     T = transform(1); 
%     T1 = transform(2); 
%     T2 = transform(3);
    
    p0 = [0 0 0];
    p1 = T1(1:3,end);
    p2 = T1*T2; p2 = p2(1:3,end);
    p3 = T(1:3,end);
    
    hold on;
    
    % Plot each link on a 3D plot 
    if isempty(lastModel)
    	figure(1);
        % ch = checkerboard(10, 16) > .5;
        %         H(1) = slice(repmat(double(ch),[1 1 2]),[],[],1);
        
        
        hp1 = plot3([p0(1) p1(1)], [p0(2) p1(2)], [p0(3) p1(3)], 'b', 'LineWidth', 2);
        hp2 = plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'g', 'LineWidth', 2);
        hp3 = plot3([p2(1) p3(1)], [p2(2) p3(2)], [p2(3) p3(3)], 'r', 'LineWidth', 2);
        
        if nargin > 4
            hq = quiver3(p3(1), p3(2), p3(3), p_dot(1), p_dot(2), p_dot(3), 'm', 'LineWidth', 2);
        end
        
        if nargin > 5
%             he = plot_ellipse(A, p3, 'edgecolor', 'k', 'fillcolor', 'c', 'alpha', 0.5);
            he = plot_ellipse(A, p3, 'edgecolor', 'k', 'fillcolor', 'c');
        end
        
        if nargin > 7
            hwa = annotation('textbox', [.65 .85 .1 .1], 'String', "", 'color', 'r', 'LineStyle', 'none', 'FitBoxToText','off');
%             if manipEllipseVol < singularityThreshold
%                 hwa.String = "Ellipse Volume: " + manipEllipseVol + newline + "Warning:" + newline + "Near singularity!";
%                 hwa.Color = 'r';
%             else
%                 hwa.String = "Ellipse Volume: " + manipEllipseVol;
%                 hwa.Color = 'b';
%             end
        end
        
        grid on;
        view(45,28);
        axis equal;
%         xlim([-50 300]), ylim([-250 250]), zlim([-100 400]);
        xlim([-400 400]), ylim([-400, 400]), zlim([-400, 600]);
        xlabel('X [mm]');
        ylabel('Y [mm]');
        zlabel('Z [mm]');
        title('Stick Model');
        
        ht1 = triad('Scale', 30, 'LineWidth', 1, 'Matrix', T);
        ht2 = triad('Scale', 30, 'LineWidth', 1, 'Matrix', T1);
        ht3 = triad('Scale', 30, 'LineWidth', 1, 'Matrix', T1*T2);
        
        stickModel = [ht1, ht2, ht3, hp1, hp2, hp3];
        
        if nargin > 4
            stickModel = [stickModel hq];
        end
        
        if nargin > 5
            stickModel = [stickModel he];
        end
        
        if nargin > 7
            stickModel = [stickModel hwa];
        end
        
        % Triads print a TON of warnings without turning these two types off
%         warning('off', 'MATLAB:hg:DiceyTransformMatrix'); 
%         warning('off', 'MATLAB:gui:array:InvalidArrayClass');
    else
        % TODO: Check if old handles are valid
%         if(ishandle(lastModel(4))...
        hp1 = lastModel(4);
        hp2 = lastModel(5);
        hp3 = lastModel(6);
        hp1.XData = [p0(1) p1(1)]; hp1.YData = [p0(2) p1(2)]; hp1.ZData = [p0(3) p1(3)];
        hp2.XData = [p1(1) p2(1)]; hp2.YData = [p1(2) p2(2)]; hp2.ZData = [p1(3) p2(3)];
        hp3.XData = [p2(1) p3(1)]; hp3.YData = [p2(2) p3(2)]; hp3.ZData = [p2(3) p3(3)];
        
        if nargin > 4
            hq = lastModel(7);
            hq.XData = p3(1); hq.YData = p3(2); hq.ZData = p3(3);
            hq.UData = p_dot(1); hq.VData = p_dot(2); hq.WData = p_dot(3);
        end
        
        if nargin > 5
            he = lastModel(8);
            plot_ellipse(A, p3, 'edgecolor', 'k', 'fillcolor', 'c', 'alter', he, 'alpha', 0);
        end
        
        if nargin > 7
            hwa = lastModel(9);
            if manipEllipseVol < singularityThreshold
                hwa.String = "Ellipse Volume: " + manipEllipseVol + newline + "Warning:" + newline + "Near singularity!";
                hwa.Color = 'r';
%             else
%                 hwa.String = "Ellipse Volume: " + manipEllipseVol;
%                 hwa.Color = 'b';
            end
        end
        
%         set(lastModel(1), T)
%         ht1 = lastModel(1);
%         ht1.Matrix = T;
%         ht2 = lastModel(2);
%         ht2.Matrix = T1;
%         ht3 = lastModel(3);
%         ht3.Matrix = T1*T2;
        
%         set(lastModel(1), 'Matrix', T);
%         set(lastModel(2), 'Matrix', T1);
%         set(lastModel(3), 'Matrix', T1*T2);

        stickModel = lastModel;
    end
    
    drawnow;
end