clear all;
close all;

covariance = [4.9275, 4.0022;
              4.0022, 4.8333];

mean = [0.2479, 0.3005];

[k, m] = uncertainty_ellipse(mean, covariance);
plot(k, m);
