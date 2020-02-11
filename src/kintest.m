function q = kintest(enc)

    rad = enc2rad([enc(1), enc(2), enc(3)])

    [T, T1, T2, ~] = fwkin3001(-rad);
    
    stickModel(T, T1, T2, []);
    
    fw = T(1:3,end).'
    
    q = ikin(fw)
    
    stickModelRad(q);
end