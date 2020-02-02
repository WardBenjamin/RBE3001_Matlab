function ai = CuPolSolve(t0, tf, vi, vf, thetai, thetaf)
    t = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];
    q = [thetai vi thetaf vf];
    ai = inv(t)*q;
end