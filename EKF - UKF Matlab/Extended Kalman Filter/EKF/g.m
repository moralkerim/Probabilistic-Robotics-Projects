function [s1, s2, s3] = g(mu_t1, d_t, alpha_t)
    s1 = mu_t1(1) + d_t*cos(mu_t1(3));
    s2 = mu_t1(2) + d_t*sin(mu_t1(3));
    s3 = mu_t1(3) + alpha_t;
end

