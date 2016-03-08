
%system('../_build/dna');

%data = load('../data/Visualize_Default.dna');
data = load('Distance/debug/chain');
N = size(data,1);
plot_steps = 1;
animate = false;

lim = floor(N^(1/3));
dt = 0.01;
dn = floor(N/plot_steps);
dn = 1;
data(N,:);

sp=15;
spans = [-sp, sp, -sp, sp, -sp, sp];
tic
[Nc , reduced_globule] = cut_globule(data,spans);
toc
if animate
for i = (dn+1):dn:N
    
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3));
    hold on;
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3),'.');
    axis([-lim,lim, -lim,lim, -lim,lim]) 
    pause(dt)

end
else
    figure(1)
    nrColors = 6;
    c = prism(nrColors);
    dp = floor(N/nrColors);
    for i =[1:nrColors]
        hold on
        plot3(data((i-1)*dp+1:i*dp,1),data((i-1)*dp+1:i*dp,2),...
            data((i-1)*dp+1:i*dp,3),'.-', 'color', c(i,:), ...
            'LineWidth',3);
        hold off
    end
    
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
    
    figure(2)
    for k = 1:Nc
        
    p_gl = reduced_globule{k};
    N2 = size(p_gl,1);
    hold on
    plot3(p_gl(:,1),p_gl(:,2), p_gl(:,3),'.-','LineWidth',3);
    hold off
    dp = floor(N2/nrColors);
%     for i =[1:nrColors]
%         hold on
%         plot3(p_gl((i-1)*dp+1:i*dp,1),p_gl((i-1)*dp+1:i*dp,2),...
%             p_gl((i-1)*dp+1:i*dp,3),'.-', 'color', c(i,:), ...
%             'LineWidth',3);
%         hold off
%     end
    end
    
    
    
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    %axis([-lim,lim, -lim,lim, -lim,lim])
end
