
%system('../_build/dna');

data = load('../data/Visualize_Default.dna');
N = size(data,1);
plot_steps = 1;
animate = true;


lim = floor(N^(1/2));
dt = 0.01;
dn = floor(N/plot_steps);
dn = 10
data(N,:);

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
    %axis([-lim,lim, -lim,lim, -lim,lim]) 
end
xlabel('X')
ylabel('Y')
zlabel('Z')
