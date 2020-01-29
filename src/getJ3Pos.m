function pos_j3 = getJ3Pos(theta_j1, theta_j2, theta_j3)
    L = [135, 170, 169.28]; %Lengths of links
    
    % Set up a symbolic generic transform matrix
    theta = sym('theta');
    d = sym('d');
    a = sym('a');
    alpha = sym('alpha');
    T = [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta); sin(theta), cos(alpha)*cos(theta), -cos(theta)*sin(alpha), a*sin(theta); 0, sin(alpha), cos(alpha), d; 0, 0, 0, 1];
    
    % Calculate three transformation matrices
    T1 = single(subs(T,{theta, d, a, alpha}, {theta_j1, L(1), 0, -pi/2}));
    T2 = single(subs(T,{theta, d, a, alpha}, {theta_j2, 0, L(2), 0}));
    
    % Calculate composite transform vectors
    T_final = T1 * T2;
    
    % Calculate position array
    pos = T_final * [theta_j1; theta_j2; theta_j3; 1];
    
    % Return values from position array
    pos_j3 = pos(1:3);
    
end