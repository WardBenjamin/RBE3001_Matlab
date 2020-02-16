coms = initialize();

pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);
pid_config(coms, [0 0 0],[0 0 0],[0 0 0]);

model = stickModel(eye(4), eye(4), eye(4), [], zeros(1, 3), eye(3), 0, 0);

% model = stickModel(eye(4), eye(4), eye(4), []);

runTime = 60;

tic
while 1
    if toc > runTime
        break;
    end
    returnPacket = status(coms);
    q = -enc2rad(returnPacket(1:3));
    q_dot = -enc2rad(returnPacket(4:6));
    
    [T, T1, T2, ~] = fwkin(q);
    [p_dot, j0, jp, ~] = fwkindiff(q, q_dot);
    A = jp*jp.';
    % https://math.stackexchange.com/questions/1970471/find-the-volume-of-the-ellipsoid-11x29y215z2-4xy10yz-20xz-80
    manipEllipseVol = (4/3)*pi / sqrt(det(A^2));
%     prod(eig(inv(A)))^(-1/2)*4/3*pi
%     prod(ei   g(A))^(-1/2)*4/3*pi
    % ?????? 
    % TODO: How do we actually calculate the volume of this ellipsoid? These
    % methods seem to give garbage answers. Not sure if A is "A" or "inv(A)"
    singularityThreshold = 4; %TODO: What should this be? Need to test.
    
    stickModel(T, T1, T2, model, p_dot, A, manipEllipseVol, singularityThreshold);
end

coms.shutdown();