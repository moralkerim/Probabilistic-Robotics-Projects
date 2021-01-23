function [mu_t_predicted, sigma_t_predicted, X] = ukf_prediction(mu_t1, sigma_t1, d, alpha)
    
    n = 2;
    p = 10;
    a = 0.5;
    beta = 2;
    lambda = (a^2)*(p + n) - n;
    
    Wm = 1:5;
    Wc = 1:5;
    X = zeros(3, 5);
    
    Rt = [0.001, 0, 0; 0, 0.001, 0; 0, 0, 0.001]; %zeros(3, 3);
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
    
    %the prediction equations
    %sigma points
    sigma_sub = sqrtm((n + lambda)*sigma_t1);
    
    X(:,1) = mu_t1;
    X(:,2) = mu_t1 + sigma_sub(:,1);
    X(:,3) = mu_t1 + sigma_sub(:,2);
    X(:,4) = mu_t1 - sigma_sub(:,1);
    X(:,5) = mu_t1 - sigma_sub(:,2);
    
    %mean and covariance predictions
    for k = 1:5
        [s1, s2, s3] = g(X(:,k), d, alpha);
        gf = [s1; s2; s3];
        mu_sum = mu_sum + Wm(k)*gf;
    end
    mu_t_predicted = mu_sum;
    
    for k = 1:5
        [s1, s2, s3] = g(X(:,k), d, alpha);
        gf = [s1; s2; s3];
        cov_sub = (gf - mu_t_predicted)*(gf - mu_t_predicted)';
        sigma_sum = sigma_sum + Wc(k)*cov_sub + Rt;
    end
    sigma_t_predicted = sigma_sum;

end

