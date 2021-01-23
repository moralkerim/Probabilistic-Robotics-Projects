sample_n = 150;
x=zeros(sample_n,8); y=zeros(sample_n,8); tet = zeros(sample_n,8);
x(:,1) = 1; tet(:,1) = pi/2;
r = 1;
t = 1; %time of traviling for 45 degrees
d2r = pi/180;
w = -pi/4; %Assumed 45 degrees travel in one sec.
v = w * r;


a1 = 0.001;  a2 = 0.001;
a3 = 0.001;  a4 = 0.001;
a5 = 0.001;  a6 = 0.001;
plot(x(1,1),y(1,1),'k.','LineWidth',2);
hold on;
    for i=1:7
        for j=1:sample_n
            v_hat = v + normal_dist(a1, a2, v,w);
            w_hat = w + normal_dist(a3, a4, v,w);
            gama_hat = normal_dist(a5, a6, v,w);
            x(j,i+1) = x(j,i) -  v_hat/w_hat * sin(tet(j,i)) +  v_hat/w_hat * sin(tet(j,i) + w_hat*t);
            y(j,i+1) = y(j,i) +  v_hat/w_hat * cos(tet(j,i)) -  v_hat/w_hat * cos(tet(j,i) + w_hat*t);
            tet(j,i+1) = tet(j,i) + w_hat*t + gama_hat*t;
            plot(x(j,i+1),y(j,i+1),'k.','LineWidth',2);
            hold on;
        end
    end


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