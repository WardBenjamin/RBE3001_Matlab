coms = initialize();

pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);
pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);

model = stickModel(eye(4), eye(4), eye(4), [], zeros(1, 3));


while 1
    returnPacket = status(coms);
    q = -enc2rad(returnPacket(1:3));
    q_dot = -enc2rad(returnPacket(4:6));
    
    [T, T1, T2, ~] = fwkin(q);
    
    p_dot = fwkindiff(q, q_dot);

    stickModel(T, T1, T2, model, p_dot);
end