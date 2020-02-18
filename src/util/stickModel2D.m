function stickModel2D(T, T1, T2)
    
    p0 = [0 0 0];
    p1 = T1(1:3,end);
    p2 = T1*T2; p2 = p2(1:3,end);
    p3 = T(1:3,end);
    
    plot([p0(1) p1(1)], [p0(3) p1(3)], 'b', 'LineWidth', 2);
    hold on
    plot([p1(1) p2(1)], [p1(3) p2(3)], 'g', 'LineWidth', 2);
    plot([p2(1) p3(1)], [p2(3) p3(3)], 'r', 'LineWidth', 2);
    
    xlim([-400, 400]);
    ylim([-40, 400]);
end