function [x_eff, y_eff, z_eff] = fwkin3001(theta_j1, theta_j2, theta_j3)
    % Set up a symbolic generic transform matrix
    syms theta d a alpha;
    T = [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta); sin(theta), cos(alpha)*cos(theta), -cos(theta)*sin(alpha), a*sin(theta); 0, sin(alpha), cos(alpha), d; 0, 0, 0, 1];
    
    % Calculate three transformation matrices
    T1 = single(subs(T,{theta, d, a, alpha}, {theta_j1, 0, 135, -pi/2}));
    T2 = single(subs(T,{theta, d, a, alpha}, {theta_j2, 0, 175, 0}));
    T3 = single(subs(T,{theta, d, a, alpha}, {theta_j3, 0, 169.28, 0}));
    
    T_final = T1 * T2 * T3;
    
end

