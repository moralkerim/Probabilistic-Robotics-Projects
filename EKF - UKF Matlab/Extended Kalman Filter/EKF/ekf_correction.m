function [mu_t_corrected, sigma_t_corrected] = ekf_correction(mu_t_predicted, sigma_t_predicted, x_lm, y_lm, s_lm, r, phi, sign, vr, vp, vs)
    
    Qt = [vr, 0, 0; 0, vp, 0; 0, 0, vs];
    
    %the correction equations
    dx = x_lm - mu_t_predicted(1);
    dy = y_lm - mu_t_predicted(2);
    q = sqrt(dx^2 + dy^2);
    
    [s1, s2, s3] = h_ekf(q, dx, dy, mu_t_predicted(3), s_lm);
    hf = [s1; s2; s3];
    Ht = [dx*sqrt(q), -dy*sqrt(q), 0; dy, dx, -1; 0, 0, 0]*(1/q);
    z = [r; phi; sign];
    Kt = (sigma_t_predicted*(Ht)')*inv(Ht*sigma_t_predicted*(Ht)' + Qt);
    Kt*(z - hf)
    mu_t_corrected = mu_t_predicted + Kt*(z - hf);
    sigma_t_corrected = (eye(3) - Kt*Ht)*sigma_t_predicted;
    
end

