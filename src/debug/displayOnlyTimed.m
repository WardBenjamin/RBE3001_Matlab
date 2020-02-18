coms = initialize();

pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);
pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);

show_ellipsoid = true;

if show_ellipsoid
    model = stickModel(eye(4), eye(4), eye(4), [], zeros(1, 3), eye(3), 0, 0);
else
    model = stickModel(eye(4), eye(4), eye(4), [], zeros(1, 3));
end

runTime = 15;

lastTime = 0;

loopTimings = [];

tic
while 1
    currentTime = toc;
    
    deltaTime = currentTime - lastTime;
    loopTimings = [loopTimings deltaTime];
    
    lastTime = currentTime;
    
    if currentTime > runTime
        break;
    end
    
    returnPacket = status(coms);
    q = -enc2rad(returnPacket(1:3));
    q_dot = -enc2rad(returnPacket(4:6));
    
    [T, T1, T2, ~] = fwkin(q);
    [p_dot, j0, jp, ~] = fwkindiff(q, q_dot);
    A = jp*jp.';

    % https://math.stackexchange.com/questions/1970471/find-the-volume-of-the-ellipsoid-11x29y215z2-4xy10yz-20xz-80
    %manipEllipseVol = (4/3)*pi / sqrt(det(A^2));
    manipEllipseVol = (4/3)*pi * sqrt(prod(eig(A)));
    %prod(eig(inv(A)))^(-1/2)*4/3*pi
    %prod(eig(A))^(-1/2)*4/3*pi

    singularityThreshold = 8*10^6; %TODO: What should this be? Need to test.
    
    if show_ellipsoid
        stickModel(T, T1, T2, model, p_dot, A, manipEllipseVol, singularityThreshold);
    else
        stickModel(T, T1, T2, model, p_dot);
    end
    
    if manipEllipseVol < singularityThreshold
        break;
    end
end

coms.shutdown();

figure(2);
histogram(loopTimings, 20);
title("Loop Timings")
xlabel("Time (seconds)")
ylabel("Occurances")