function [theta1, theta2, theta3] = ikin(x, y, z)
    L = [135, 175, 169.28]; %Lengths of links

    r = sqrt(x^2 + y^2);
    s = z - L(1);
    theta1 = pi + atan2(y,x);
    theta3 = (L(2)^2 + L(3)^2 - (r^2 + z^2))/(2*L(2)*L(3));
    theta2 = asin((s-L(3)*sin(theta3)/r));
end