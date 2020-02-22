function set_gripper(pp, open)
try
        SERV_ID = 05;            % we will be talking to server ID 05 on
        % the Nucleo

       
        DEBUG   = true;          % enables/disables debug prints
        
        % Instantiate a packet - the following instruction allocates 64
        % bytes for this purpose. Recall that the HID interface supports
        % packet sizes up to 64 bytes.
        packet = zeros(15, 1, 'single');
        if open
            packet(1) = 1;
        end
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
            disp('Sent Packet Gripper:');
            disp(packet);
            disp('Received Packet Gripper:');
            disp(returnPacket);
        end

%         toc;
%         pause(1) %timeit(returnPacket) !FIXME why is this needed?
        
        
    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
    end
end