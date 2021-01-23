%% Initialize states
dist = 1;   
N=1000;
tet = linspace(-pi,pi,N);                    %teta variable between 0 and 1
x = dist*cos(tet);                           %x variable between 0 and 1
y = dist*sin(tet);                           %y variable between 0 and 1
xt = [x; y; tet];                            %state matrix
E0 = [0.01 0 0; 0 0.01 0; 0 0 10000*pi/180]; %Sigma matrix
Mu1_ = [1; 0; 0;];                           %t=1 mean values
G1 = [1 0 0; 0 1 1; 0 0 1];                  %Jacobian for Mu0
E1_ = G1*E0*G1.';
H1 = [1 0 0];                                %Measurement Jacobian
Q1 = 0.01;                                   %Measurement noise covariance
zt = -1;
I = eye( 3 );
%% Calculate Kalman gain and posteriors
K1 = E1_*H1.'*inv(H1*E1_*H1.'+ Q1);
Mu1 = Mu1_ + K1 * (zt - 1);
E1 = (I - K1*H1) * E1_;
%% Plot
error_ellipse(E1,'mu',Mu1)
xlabel('x position'); ylabel('y position'); zlabel('teta');
clear all; clc
