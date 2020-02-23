function done(coms, cam)
    set_setpoint(coms, [0 0 0])
    coms.shutdown();
    clear cam;
end