function [T_final, T1, T2, T3] = fwkin3001(q)
    L = [135, 175, 169.28]; %Lengths of links
    
    
    T = @(theta, d, a, alpha) ... 
        [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta); 
         sin(theta), cos(alpha)*cos(theta), -cos(theta)*sin(alpha), a*sin(theta); 
         0, sin(alpha), cos(alpha), d; 
         0, 0, 0, 1];
    
%     Set up a symbolic generic transform matrix
%     theta = sym('theta');
%     d = sym('d');
%     a = sym('a');
%     alpha = sym('alpha');
%     T = [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta); sin(theta), cos(alpha)*cos(theta), -cos(theta)*sin(alpha), a*sin(theta); 0, sin(alpha), cos(alpha), d; 0, 0, 0, 1];
%     
    % Calculate three transformation matrices
    T1 = T(q(1), L(1), 0, -pi/2);
    T2 = T(q(2), 0, L(2), 0);
    T3 = T(q(3) + pi/2, 0, L(3), 0);
    
    % Calculate composite transform vectors
    T_final = T1 * T2 * T3;
    
    % Calculate position array
%      = T_final * [theta_j1; theta_j2; theta_j3; 1];
%     
%     % Return values from position array
%     pos_eff = pos(1:3);
end

