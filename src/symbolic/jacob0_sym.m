function J = jacob0_sym()

    q = [sym('theta1'), sym('theta2'), sym('theta3')];
    
    [T, T1, T2, T3] = fwkin_sym(q);
    pe = T(1:3, end);
    
    Jp = [diff(pe, q(1)), diff(pe, q(2)), diff(pe, q(3))];
    
    T02 = T1*T2;
    
    Jo = [T1(1:3, end-1), T02(1:3, end-1), T(1:3, end-1)];
    
    J = [Jp; Jo];
end