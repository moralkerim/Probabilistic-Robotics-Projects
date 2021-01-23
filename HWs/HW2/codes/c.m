%% Initialize states
d = 1;   
N=1000;
tet = linspace(-pi,pi,N);             %teta variable between 0 and 1
x = d*cos(tet);                     %x variable between 0 and 1
y = d*sin(tet);                        %y variable between 0 and 1
xt = [x; y; tet];                     %state matrix
E0 = [0.01 0 0; 0 0.01 0; 0 0 10000*pi/180]; %Sigma matrix
Mu1_ = [1; 0; 0;];                   %t=1 mean values
G1 = [1 0 0; 0 1 1; 0 0 1];          %       %Jacobian for Mu0
E1_ = G1*E0*G1.';
px = zeros(1,N);
%% Draw Uncertianty Ellipse
error_ellipse(E1_,'mu',Mu1_)
xlabel('x position'); ylabel('y position'); zlabel('teta');
clear all; clc