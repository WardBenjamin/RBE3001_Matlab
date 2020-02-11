function set_position(coms, p)
    q = ikin(p)
    set_setpoint(coms, rad2enc([q(1), -q(2), q(3)])); %%TODO: set_setpoints behavior not as expected
end