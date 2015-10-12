

%system('../_build/dna ax 200');

data = load('../data/default_visualize_data.dna');
N = size(data,1);
plot_steps = 1;
animate = false;


lim = floor(N^(3/5));
dt = 0.01;
dn = floor(N/plot_steps);

data(N,:)

if animate
for i = (dn+1):dn:N
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3));
    hold on;
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3),'.');
    axis([-lim,lim, -lim,lim, -lim,lim]) 
    pause(dt)

end
else
    plot3(data(:,1),data(:,2),data(:,3),'.-');
    axis([-lim,lim, -lim,lim, -lim,lim]) 
end
