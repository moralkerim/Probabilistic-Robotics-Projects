function [s1, s2, s3] = h_ekf(q, dx, dy, theta_t, s_lm)

    s1 = q;
    s2 = atan2(dy, dx) - theta_t;
    s3 = s_lm;
end
