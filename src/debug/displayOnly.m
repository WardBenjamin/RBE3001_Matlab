coms = initialize();

pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);
pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);

% model = stickModel(eye(4), eye(4), eye(4), zeros(1, 3), []);

model = stickModelBasic(eye(4), eye(4), eye(4), []);


while 1
    returnPacket = status(coms);
    q = -enc2rad(returnPacket(1:3));
    q_dot = -enc2rad(returnPacket(4:6));
    
    [T, T1, T2, ~] = fwkin(-enc2rad(q));
    stickModelBasic(T, T1, T2, model);
    
    p_dot = fwkindiff(q, q_dot);

%     model = stickModel(T, T1, T2, p_dot, model);
end