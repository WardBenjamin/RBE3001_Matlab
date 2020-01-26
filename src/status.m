function returnPacket = status(pp)
    try
        SERV_ID = 03;            % we will be talking to server ID 03 on
        % the Nucleo

        % Create csv file to print data to
        csvfile = fopen(sprintf('log_%s.csv', datestr(now, 'mm-dd-yyyy_HH-MM-SS')), 'a');
        fprintf(csvfile, 'Encoder_Joint1,Encoder_Joint2,Encoder_Joint3,Velocity_Joint1,Velocity_Joint2,Velocity_Joint3,\n');


        DEBUG   = true;          % enables/disables debug prints

        % Instantiate a packet - the following instruction allocates 64
        % bytes for this purpose. Recall that the HID interface supports
        % packet sizes up to 64 bytes.
        packet = zeros(15, 1, 'single');

            tic
        
            % Send packet to the server and get the response
            %pp.write sends a 15 float packet to the micro controller
            pp.write(SERV_ID, packet);

            pause(0.003); % Minimum amount of time required between write and read

            %pp.read reads a returned 15 float backet from the nucleo.
            returnPacket = pp.read(SERV_ID);
            toc

            if DEBUG
                disp('Sent Packet:');
                disp(packet);
                disp('Received Packet:');
                disp(returnPacket);
            end

            fprintf(csvfile, '%f,%f,%f,%f,%f,%f,\n', returnPacket);

            toc
            pause(1) %timeit(returnPacket) !FIXME why is this needed?

        
        fclose(csvfile);


    catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
    end
end
