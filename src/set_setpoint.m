function returnPacket = set_setpoint(pp, setpoints)
    try
        SERV_ID = 01;            % we will be talking to server ID 03 on
        % the Nucleo

       
        DEBUG   = true;          % enables/disables debug prints

        % Instantiate a packet - the following instruction allocates 64
        % bytes for this purpose. Recall that the HID interface supports
        % packet sizes up to 64 bytes.
        packet = zeros(15, 1, 'single');
        packet(1) = setpoints(1);
        packet(4) = setpoints(2);
        packet(7) = setpoints(3);
        packet(15) = 1; % Wakeup gate

%         tic;

        % Send packet to the server and get the response
        %pp.write sends a 15 float packet to the micro controller
        pp.write(SERV_ID, packet);

        pause(0.003); % Minimum amount of time required between write and read

        %pp.read reads a returned 15 float backet from the nucleo.
        returnPacket = pp.read(SERV_ID);
%         toc;

        if DEBUG
            disp('Sent Packet Setpoint:');
            disp(packet);
            disp('Received Packet Setpoint:');
            disp(returnPacket);
        end

%         toc;
%         pause(1) %timeit(returnPacket) !FIXME why is this needed?
        
        
    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
    end
end
