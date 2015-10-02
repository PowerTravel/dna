

system('../_build/dna 10000');

data = load('../data/initial_state.m');
N = size(data,1)
plot_steps = N;
animate = false;


lim = floor(sqrt(N));
dt = 0.01;
dn = floor(N/plot_steps)

if animate
for i = (dn+1):dn:N
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3));
    hold on;
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3),'.');
    axis([-lim,lim, -lim,lim, -lim,lim]) 
    pause(0.01)
end
else
    plot3(data(:,1),data(:,2),data(:,3),'.-');
    axis([-lim,lim, -lim,lim, -lim,lim]) 
end