function [rad] = enc2rad(encoder_value)
    rad = encoder_value / 4096.0 * 2*pi;
end

