
%system('../_build/dna');

data = load('../data/Visualize_Default.dna');
N = size(data,1);
plot_steps = 1;
animate = false;

lim = floor(N^(1/3));
dt = 0.01;
dn = floor(N/plot_steps);
dn = 1;
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
    nrColors = 5;
    c = prism(nrColors);
    dp = floor(N/nrColors);
    for i =[1:nrColors]
        hold on
        plot3(data((i-1)*dp+1:i*dp,1),data((i-1)*dp+1:i*dp,2),...
            data((i-1)*dp+1:i*dp,3),'.-', 'color', c(i,:), ...
            'LineWidth',3);
    end
    %axis([-lim,lim, -lim,lim, -lim,lim])
end
xlabel('X')
ylabel('Y')
zlabel('Z')
