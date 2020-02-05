coms = initialize();

calibrate(coms)

pid_config(coms, [.0007, .0004, 0], [.0009, .0002, .1], [.005, .0001, 0])
pid_config(coms, [.0007, .0004, 0], [.0009, .0002, .1], [.005, .0001, 0])

set_setpoint(coms, [0 0 0]);
set_setpoint(coms, [0 rad2enc(.5) rad2enc(0)]);

while 1
end

% shutdown(coms);