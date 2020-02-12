coms = initialize();

pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);
pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);

model = stickModel(eye(4), eye(4), eye(4), []);

while 1
    position = status(coms);
    [T, T1, T2, ~] = fwkin3001(-enc2rad(position));
    stickModel(T, T1, T2, model);
end