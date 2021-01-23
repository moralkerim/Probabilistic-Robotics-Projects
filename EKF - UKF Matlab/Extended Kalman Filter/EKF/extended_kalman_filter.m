%KOM613E Homework 2

%Initial beliefs
mu_t1 = [0; 0; 0];
sigma_t1 = [0.01, 0, 0; 0, 0.01, 0; 0, 0, 10000];

%input
dx = 3;
dy = 0;
dt = 0;

%measurement
zx = 3.8;
zy = 0.00001;
zt = 0.5;

[mu_t, sigma_t] = EKF(mu_t1(1), mu_t1(2), mu_t1(3), sigma_t1(1,1), sigma_t1(1,2), sigma_t1(1,3), sigma_t1(2,1), sigma_t1(2,2), sigma_t1(2,3), sigma_t1(3,1), sigma_t1(3,2), sigma_t1(3,3), dx, dy, dt, zx, zy, zt);

%now let us draw uncertainty ellipse for our intuitive posterior
%z = 1 in this case
mu_reduced = [mu_t(1); mu_t(2)];
sigma_reduced = [sigma_t(1, 1), sigma_t(1, 2);
                 sigma_t(2, 1), sigma_t(2, 2)];
error_ellipse(sigma_reduced, mu_reduced);


