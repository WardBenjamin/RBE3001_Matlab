function done(coms)
    set_setpoint(coms, [0 0 0])
    coms.shutdown();
end