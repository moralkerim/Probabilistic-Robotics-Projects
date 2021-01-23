function [mu_t_predicted, sigma_t_predicted] = ekf_prediction(mu_t1, sigma_t1, d, alpha)
    
    Rt = [0.001, 0, 0; 0, 0.001, 0; 0, 0, 0.001]; %zeros(3, 3);
    Gt = [1, 0, -d*sin(mu_t1(3)); 0, 1, d*cos(mu_t1(3)); 0, 0, 1];
    
    %the prediction equations
    [x_prd, y_prd, theta_prd] = g(mu_t1, d, alpha);
    mu_t_predicted = [x_prd; y_prd; theta_prd];
    sigma_t_predicted = Gt*sigma_t1*(Gt)' + Rt;

end

