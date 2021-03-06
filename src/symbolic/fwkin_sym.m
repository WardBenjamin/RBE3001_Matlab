function [T_final, T1, T2, T3] = fwkin_sym(q)
    L = [sym('L1'), sym('L2'), sym('L3')];

    T = @(theta, d, a, alpha) ... 
        [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta); ...
        sin(theta), cos(alpha)*cos(theta), -cos(theta)*sin(alpha), a*sin(theta); ...
        0, sin(alpha), cos(alpha), d; ...
        0, 0, 0, 1];
    
    % Calculate three transformation matrices
    T1 = T(q(1), L(1), 0, -pi/2);
    T2 = T(q(2), 0, L(2), 0);
    T3 = T(q(3) + pi/2, 0, L(3), 0);
    
    % Calculate composite transform vectors
    T_final = T1 * T2 * T3;
end