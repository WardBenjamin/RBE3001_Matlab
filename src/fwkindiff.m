function [p_dot, j0, j0p, j0o] = fwkindiff(q, q_dot)
    j0 = jacob0(q);
    j0p = j0(1:3, 1:end);
    j0o = j0(4:end, 1:end);
    p_dot = j0 * q_dot;
end