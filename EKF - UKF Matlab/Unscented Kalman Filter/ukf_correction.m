function [mu_t_corrected, sigma_t_corrected] = ukf_correction(mu_t_predicted, sigma_t_predicted, X, d, alpha, x_lm, y_lm, s_lm, r, phi, sign, vr, vp, vs)
    
    n = 2;
    p = 10;
    a = 0.5;
    beta = 2;
    lambda = (a^2)*(p + n) - n;
    
    Wm = 1:5;
    Wc = 1:5;
    L = zeros(3, 5);
    
    Qt = [vr, 0, 0; 0, vp, 0; 0, 0, vs];
    mu_sum = zeros(3,1);
    sigma_sum = zeros(3,3);
    
    %mean and covariance weigths
    Wm(1) = lambda/(n + lambda);
    Wm(2) = 1/(2*(n + lambda));
    Wm(3) = Wm(2);
    Wm(4) = Wm(2);
    Wm(5) = Wm(2);
    
    Wc(1) = lambda/(n + lambda) + 1 - a^2 + beta;
    Wc(2) = 1/(2*(n + lambda));
    Wc(3) = Wc(2);
    Wc(4) = Wc(2);
    Wc(5) = Wc(2);
    
    %the correction equations
    %sigma points
    sigma_sub = sqrtm((n + lambda)*sigma_t_predicted);
    
    L(:,1) = mu_t_predicted;
    L(:,2) = mu_t_predicted + sigma_sub(:,1);
    L(:,3) = mu_t_predicted + sigma_sub(:,2);
    L(:,4) = mu_t_predicted - sigma_sub(:,1);
    L(:,5) = mu_t_predicted - sigma_sub(:,2);
    
    %mean and covariance predictions
    for k = 1:5
        [s1, s2, s3] = h_ukf(L(:,k), x_lm, y_lm, s_lm);
        hf = [s1; s2; s3];
        mu_sum = mu_sum + Wm(k)*hf;
    end
    z_t_bar = mu_sum;
    
    for k = 1:5
        [s1, s2, s3] = h_ukf(L(:,k), x_lm, y_lm, s_lm);
        hf = [s1; s2; s3];
        cov_sub = (hf - z_t_bar)*(hf - z_t_bar)';
        sigma_sum = sigma_sum + Wc(k)*cov_sub + Qt;
    end
    S_t = sigma_sum;
    sigma_sum = zeros(3,3);
    
    %cross covariance
    for k = 1:5
        [s1, s2, s3] = g(X(:,k), d, alpha);
        gf = [s1; s2; s3];
        [s1, s2, s3] = h_ukf(L(:,k), x_lm, y_lm, s_lm);
        hf = [s1; s2; s3];
        
        cov_sub = (gf - mu_t_predicted)*(hf - z_t_bar)';
        sigma_sum = sigma_sum + Wc(k)*cov_sub;
    end
    sigma_cross = sigma_sum;
    
    %gain
    K_t = sigma_cross*inv(S_t);
    
    %correct mean
    z_t = [r; phi; sign];
    mu_t_corrected = mu_t_predicted + K_t*(z_t - z_t_bar);
    
    %correct covariance
    sigma_t_corrected = sigma_t_predicted - K_t*(sigma_cross)';
    
end

    