function q = ikin(p)
    L = [135, 175, 169.28]; %Lengths of links
    
    x = p(1);
    y = p(2);
    z = p(3);

%     r = sqrt(x^2 + y^2);
%     s = z - L(1);
%     %theta1 = pi + atan2(y,x);
%     theta3_numerator = (L(2)^2 + L(3)^2 - (r^2 + s^2));
%     theta3_denominator = (L(2) * L(3));
%     theta3 = acos(-theta3_numerator / theta3_denominator) + pi/2;
%     theta2 = atan((s + L(3) * tan(pi/2 - theta3)) / r);
    
    theta1 = atan2(y,x);
    
    a = x^2 + y^2 + (z - L(1))^2;
    
    theta3_n = L(2)^2 + L(3)^2 - a;
    theta3_d = 2 * L(2) * L(3);
    theta3 = -(acos(theta3_n / theta3_d) - pi/2);
    
     alpha = atan2((z - L(1)), sqrt(x^2 + y^2));
%     alpha = asin((z - L(1)) / a);
    
    beta_n = L(2)^2 + a - L(3)^2;
    beta_d = 2 * L(2) * sqrt(a);
    beta = acos(beta_n / beta_d);
    
    theta2 = -(alpha + beta);
    
    q = [theta1 theta2 theta3];
end