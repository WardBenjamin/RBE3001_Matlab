function p_dot = fwkindiff(q, q_dot)
    p_dot = jacob0(q) * q_dot;
end