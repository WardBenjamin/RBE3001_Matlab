function calibrate(coms)
    statusPacket = status(coms);
    pause(.1);
    statusPacket = status(coms);
    pause(.1);
    calibration(coms, statusPacket);
    pause(.1);
    calibration(coms, statusPacket);
    statusPacket = status(coms);
    statusPacket = status(coms);
    statusPacket = status(coms);
end