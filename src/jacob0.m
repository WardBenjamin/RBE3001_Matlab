function J_final = jacob0(q)


%     J = @(theta1, theta2, theta3, L1, L2, L3)...
%         [ - L3*cos(theta3 + pi/2)*((4967757600021511*cos(theta1)*sin(theta2))/81129638414606681695789005144064 + cos(theta2)*sin(theta1)) - (4967757600021511*L2*cos(theta1)*sin(theta2))/81129638414606681695789005144064 - L2*cos(theta2)*sin(theta1) - L3*sin(theta3 + pi/2)*((4967757600021511*cos(theta1)*cos(theta2))/81129638414606681695789005144064 - sin(theta1)*sin(theta2)), - L3*cos(theta3 + pi/2)*(cos(theta1)*sin(theta2) + (4967757600021511*cos(theta2)*sin(theta1))/81129638414606681695789005144064) - L2*cos(theta1)*sin(theta2) - (4967757600021511*L2*cos(theta2)*sin(theta1))/81129638414606681695789005144064 - L3*sin(theta3 + pi/2)*(cos(theta1)*cos(theta2) - (4967757600021511*sin(theta1)*sin(theta2))/81129638414606681695789005144064), - L3*cos(theta3 + pi/2)*(cos(theta1)*sin(theta2) + (4967757600021511*cos(theta2)*sin(theta1))/81129638414606681695789005144064) - L3*sin(theta3 + pi/2)*(cos(theta1)*cos(theta2) - (4967757600021511*sin(theta1)*sin(theta2))/81129638414606681695789005144064); ...
%         L2*cos(theta1)*cos(theta2) + L3*cos(theta3 + pi/2)*(cos(theta1)*cos(theta2) - (4967757600021511*sin(theta1)*sin(theta2))/81129638414606681695789005144064) - L3*sin(theta3 + pi/2)*(cos(theta1)*sin(theta2) + (4967757600021511*cos(theta2)*sin(theta1))/81129638414606681695789005144064) - (4967757600021511*L2*sin(theta1)*sin(theta2))/81129638414606681695789005144064,   (4967757600021511*L2*cos(theta1)*cos(theta2))/81129638414606681695789005144064 + L3*cos(theta3 + pi/2)*((4967757600021511*cos(theta1)*cos(theta2))/81129638414606681695789005144064 - sin(theta1)*sin(theta2)) - L3*sin(theta3 + pi/2)*((4967757600021511*cos(theta1)*sin(theta2))/81129638414606681695789005144064 + cos(theta2)*sin(theta1)) - L2*sin(theta1)*sin(theta2),   L3*cos(theta3 + pi/2)*((4967757600021511*cos(theta1)*cos(theta2))/81129638414606681695789005144064 - sin(theta1)*sin(theta2)) - L3*sin(theta3 + pi/2)*((4967757600021511*cos(theta1)*sin(theta2))/81129638414606681695789005144064 + cos(theta2)*sin(theta1));...
%         0, L3*sin(theta2)*sin(theta3 + pi/2) - L3*cos(theta2)*cos(theta3 + pi/2) - L2*cos(theta2), L3*sin(theta2)*sin(theta3 + pi/2) - L3*cos(theta2)*cos(theta3 + pi/2);...
%         -sin(theta1), -sin(theta1), -sin(theta1);...
%         cos(theta1), cos(theta1), cos(theta1);...
%         4967757600021511/81129638414606681695789005144064, 4967757600021511/81129638414606681695789005144064, 4967757600021511/81129638414606681695789005144064]

    J = @(theta1, theta2, theta3, L1, L2, L3)...
        [L3*sin(theta1)*cos(theta2)*sin(theta3)+L3*sin(theta1)*sin(theta2)*cos(theta3) - L2*sin(theta1)*cos(theta2), L3*cos(theta1)*sin(theta2)*sin(theta3)-L3*cos(theta1)*cos(theta2)*cos(theta3)-L2*cos(theta1)*sin(theta2), -L3*cos(theta1)*cos(theta2)*cos(theta3)+L3*cos(theta1)*sin(theta2)*sin(theta3);...
        -L3*cos(theta1)*cos(theta2)*sin(theta3)-L3*cos(theta1)*sin(theta2)*cos(theta3) + L2*cos(theta1)*cos(theta2), L3*sin(theta1)*sin(theta2)*sin(theta3)-L3*sin(theta1)*cos(theta2)*cos(theta3)-L2*sin(theta1)*sin(theta2), -L3*sin(theta1)*cos(theta2)*cos(theta3)+L3*sin(theta1)*sin(theta2)*sin(theta3);...
        0, L3*cos(theta2)*sin(theta3)+L3*sin(theta2)*cos(theta3)-L2*cos(theta2), L3*sin(theta2)*cos(theta3)+L3*cos(theta2)*sin(theta3);...
        -sin(theta1), -sin(theta1), -sin(theta1);...
        cos(theta1), cos(theta1), cos(theta1);...
        0, 0, 0];

%     [~, T1, T2, T3] = fwkin(q);
%     Jo = [T1(1:3, end-1), T2(1:3, end-1), T3(1:3, end-1)];
%     
%     J = [Jp; Jo];

    J_final = J(q(1), q(2), q(3), 135, 175, 169.28);
end