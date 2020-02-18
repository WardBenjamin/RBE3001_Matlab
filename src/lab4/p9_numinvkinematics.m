clear;
cla;
clf;
close all;

coms = initialize();

q0 = [0; 0; 0];
epsilon = 1*10^-2;

[T, T1, T2, ~] = fwkin(q0);
stickModel2D(T, T1, T2);

qi = q0;

% qf = ikin(pd)
% [T, T1, T2, ~] = fwkin(qf);
% stickModel2D(T, T1, T2);

itr = 0;

for i = 1:10
    [px, pz] = ginput(1);
    plot(px, pz, 'o');

    pd = [px; 0; pz];

    while 1
        [T, T1, T2, ~] = fwkin(qi); 
        fqi = T(1:3, end);

        error = pd - fqi;
        if norm(error) < epsilon || itr > 25
            break;
        end

        j0 = jacob0(qi); jp = j0(1:3,1:3);

        dq = pinv(jp) * error * .35;
        qi = qi + dq;

        stickModel2D(T, T1, T2);
        drawnow;
        
        itr = itr + 1;

%         java.lang.Thread.sleep(250);
    end
    
    if itr > 25
        itr = 0;
        continue;
    end
    
    status(coms);
    set_setpoint(coms, -rad2enc(qi));
end

coms.shutdown();
