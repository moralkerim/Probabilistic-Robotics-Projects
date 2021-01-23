%% Initialize states
d = 1;   
N=100;
tet = linspace(-pi,pi,N);                     %teta variable between 0 and 1
x =  d*cos(tet);                        %x variable between 0 and 1
y = d*sin(tet);                     %y variable between 0 and 1
xt = [x; y; tet];                   %state matrix
%% Plot the positions
for i=1:N
    plot(x(i),y(i),'*');
    hold on;
end
xlabel('x position [m]'); ylabel('y position [m]');


