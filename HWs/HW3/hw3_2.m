sample_n = 150;
xc = 0; yc = 0;
x=zeros(sample_n,8); y=zeros(sample_n,8); tet = zeros(sample_n,8);
xp=zeros(sample_n,8); yp=zeros(sample_n,8); tetp = zeros(sample_n,8);
xp(:,1) = 1; yp(:,1) = 0; tetp(:,1) = -pi/2;
tet(:,1) = -pi/2;
r = 1;
t = 1; %time of traviling for 45 degrees
d2r = pi/180;
w = -pi/4; %Assumed 45 degrees travel in one sec.
v = w * r;
for i=1:8
      x(:,i) = xc - r*sin(tet(1,i));
      y(:,i) = yc + r*cos(tet(1,i));
      tet(1,i+1) = tet(1,i) + w*t;
end

a1 = 0.001;  a2 = 0.001;
a3 = 0.001;  a4 = 0.001;
a5 = 0.001;  a6 = 0.001;
%plot(x,y,'k.','LineWidth',2);
hold on;
    for i=1:7
        for j=1:sample_n
            delta_rot1 = atan2((y(j,i+1)-y(j,i)),(x(j,i+1)-x(j,i))) - tet(j,i);
            delta_trans = sqrt((y(j,i+1)-y(j,i))^2 + (x(j,i+1)-x(j,i))^2);
            delta_rot2 = tet(j,i+1) - tet(j,i) - delta_rot1;
            
            delta_rot1_hat = delta_rot1 - 0;% normal_dist(a1,a2,delta_rot1,delta_trans);
            delta_trans_hat = delta_trans - 0;% normal_dist(a3,a4,delta_trans,delta_rot1+delta_rot2);
            delta_rot2_hat = delta_rot2 - 0;% normal_dist(a1,a2,delta_rot2,delta_trans);
            
            xp(j,i+1) = xp(j,i) + delta_trans_hat * cos(tetp(j,i)+delta_rot1_hat);
            yp(j,i+1) = yp(j,i) + delta_trans_hat * sin(tetp(j,i)+delta_rot1_hat);
            tetp(j,i+1) = tetp(j,i) + delta_rot1_hat + delta_rot2_hat;
        end
    end
plot(xp,yp,'k.');

function sample = normal_dist(a1, a2, v,w)
    b = sqrt(a1 * abs(v) + a2 * abs(w));
    ran_num  = -b + (b+b).*rand(12,1);
    sum = 0;
    for i=1:12
        sum = sum + ran_num(i);
    end
    sample = 0.5 * sum;
end

function sample2 = normal_dist2(a1, a2, v,w)
    b = sqrt(a1 * abs(v) + a2 * abs(w));
    sample2 = normrnd(0,b);
end