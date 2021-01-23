clear all;
close all;

%Initial beliefs
mu_t1 = [0; 0; 0];
sigma_t1 = [0.05, 0, 0; 0, 0.05, 0; 0, 0, 0.01];

%input
d = 2; %meter
alpha = 0; %rad

range = 5; %m (range)
bearing = pi/2; %rad
resolution = 0.1; %rad
numberofmeasurements = int8(bearing/resolution);

er = 0.001;
ep = 0.001;
es = 0.001;

landmark = [];
landmark = [landmark; 0.5, 0.5];
landmark = [landmark; 2.5, 3.5];
landmark = [landmark; 4.5, 0.5];
landmark = [landmark; 6.5, -0.5];
landmark = [landmark; 8.5, 0.5];
landmark = [landmark; -1, 8];
landmark = [landmark; 7, 7];
landmark = [landmark; -7.5, 7.5];
landmark = [landmark; 4, 8];
signature = 1:length(landmark(:,1));

hold on;
scatter(landmark(:, 1), landmark(:, 2), 'black')
for i = 1:20
    
    [mu_t, sigma_t] = ekf_prediction(mu_t1, sigma_t1, d, alpha);
    
    for j = 1:length(signature)
        
        [r, phi, sign] = measurement_feature(landmark(j, 1), landmark(j, 2), signature(j), mu_t(1), mu_t(2), mu_t(3), er, ep, es);
        
        if r <= range && phi <= bearing/2 && phi >= -bearing/2
            
            [mu_t, sigma_t] = ekf_correction(mu_t, sigma_t, landmark(j, 1), landmark(j, 2), signature(j), r, phi, sign, er, ep, es);
            
            z_x = mu_t(1) + r*cos(mu_t(3) + phi);
            z_y = mu_t(2) + r*sin(mu_t(3) + phi);
            scatter(z_x, z_y, '.',  'green')
            scatter(mu_t(1), mu_t(2), '.',  'red')
            break;
        end
    end
    
    mu_t_reduced = [mu_t(1); mu_t(2)];
    sigma_t_reduced = [sigma_t(1, 1), sigma_t(1, 2);
                       sigma_t(2, 1), sigma_t(2, 2)];
    [a, b] = uncertainty_ellipse(mu_t_reduced, sigma_t_reduced);
    
    plot(a, b);
    mu_t1 = mu_t;
    sigma_t1 = sigma_t;
    
    if i == 3 || i == 7 %|| i == 15 || i == 19
        alpha = pi/2;
    else
        alpha = 0;
    end
end

