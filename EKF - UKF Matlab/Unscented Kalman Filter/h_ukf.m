function [s1, s2, s3] = h_ukf(mu_t_prd, x_lm, y_lm, s_lm)

    s1 = sqrt((x_lm - mu_t_prd(1))^2 + (y_lm - mu_t_prd(2))^2);
    s2 = atan2(y_lm - mu_t_prd(2), x_lm - mu_t_prd(1)) - mu_t_prd(3);
    s3 = s_lm;
    
end

