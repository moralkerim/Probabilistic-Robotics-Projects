function [r, phi, signature] = measurement_feature(mx, my, signature, x, y, theta, er, ep, es)
r = sqrt((mx - x)^2 + (my - y)^2) + sample_normal(er);
phi = atan2(my - y, mx - x) - theta + sample_normal(ep);
signature = signature + sample_normal(es);
end

