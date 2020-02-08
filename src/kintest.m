function kintest(enc)

    rad = enc2rad([-enc(1), enc(2), enc(3)])

    [T, ~] = fwkin3001(-enc2rad(enc));
    
    fw = T(1:3,end).'
    
    p = ikin(fw)
end