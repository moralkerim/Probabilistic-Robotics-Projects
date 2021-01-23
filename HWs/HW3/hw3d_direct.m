sample_n = 150;
xc = 0; yc = 0;
x=zeros(1,8); y=zeros(1,8); tet = zeros(1,8);
v_hat=zeros(1,8); w_hat=zeros(1,8);
xg = linspace(-3,3,10); yg = linspace(-3,3,10);
prob = zeros(1,100);
tet(1) = -pi/2;
r = 1;
t = 1; %time of traviling for 45 degrees
d2r = pi/180;
w = -pi/4; %Assumed 45 degrees travel in one sec.
v = w * r;
for i=1:8
      x(i) = xc - r*sin(tet(i));
      y(i) = yc + r*cos(tet(i));
      tet(i+1) = tet(i) + w*t;
end

a1 = 0.1;  a2 = 0.1;
a3 = 0.1;  a4 = 0.1;
a5 = 0.1;  a6 = 0.1;

    for i=1:7
        for j=1:10
            for k=1:10
                mu = 0.5 * ((x(i)-xg(k))*cos(tet(i)) + (y(i)-yg(j))*sin(tet(i))) ...
                    / ((y(i)-yg(j))*cos(tet(i)) - (x(i)-xg(k))*sin(tet(i)));
                xs = 0.5*(xg(k)+x(i)) + mu*(y(i)-yg(j));
                ys = 0.5*(yg(j)+y(i)) + mu*(xg(k)-x(i));
                rs = sqrt((x(i)-xs)^2 + (y(i)-ys)^2);
                dtet = atan2(yg(j)-ys,xg(k)-xs)-atan2(y(i)-ys,x(i)-xs);
                v_hat(i+1) = dtet/t * rs;
                w_hat(i+1) = dtet/t;
                gama_hat = (tet(i+1)-tet(i))/t - w_hat(i+1);
                prob1 = gauss(v-v_hat(i+1),a1*abs(v)+a2*abs(w))*gauss(w-w_hat(i+1),a3*abs(v)+a4*abs(w))...
                    *gauss(gama_hat,a5*abs(v)+a6*abs(w));
                plot(xg(k),prob1,'k.'); hold on;
            end
        end
    end


function prob = gauss(a,b2)
prob = 1/sqrt(2*pi*b2) * exp(-0.5 * a^2 / b2);
end
