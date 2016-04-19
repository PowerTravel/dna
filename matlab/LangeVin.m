N = 200;
H = zeros(1,N);

dt  = 0.1;
T = 100;

steps = T/dt;

v = zeros(N,steps);
x = v;

for d = 1:N
    for i = 1:steps-1
        v(d,i+1) = 2*sqrt(dt)*normrnd(0,1);
        x(d,i+1) = x(d,i) + dt*v(d,i+1);
    end
end
% 
% x2 = zeros(1,steps);
% for i = 1:steps
%     x2(i) = mean(x(:,i));
%     x2(i) = xm(i)*xm(i);
% end

t = dt:dt:T;
xm = zeros(1,steps);
x2 = xm;


for i = 2:steps
    xm(i) = mean(x(:,i).*x(:,i));
    x2(i) = xm(i);
end
figure(1)
%for i = 1:N
%    plot(t,x(i,:))
%    hold on
%end
f = @(t)2.*t;
plot(log(t),log(x2))
%hold on
%plot(t,f(t))
figure(2)
histogram(x(:,end))