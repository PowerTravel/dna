
%system('../_build/dna');

%data = load('../data/Visualize_Default.dna');
globule = load('Distance/debug/chain');
N = size(globule,1);

traj = load('Distance/debug/trajectory');
plot_traj = true;

plot_steps = 10;
animate = false;

lim = floor(N^(1/3));
dt = 0.01;
dn = floor(N/plot_steps);


%sp=3.025;
sp=10;
spans = [-sp, sp, -sp, sp, -sp, sp];
spans = [-11.5, 0.5, 22.5, 37.5, 12.5, 23.5];
tic
[Nc , reduced_globule] = cut_globule(globule,spans);
toc
if animate
    nrColors = 6;
    c = prism(nrColors);
    dp = floor(N/nrColors);
%for i = (dn+1):dn:N

 for i =[1:nrColors]

        
        sub_segment = globule((i-1)*dp+1 : i*dp,:);
        N2 = size(sub_segment,1);
        dn2 = floor(N2/plot_steps);
        
       for j = (dn2+1):dn2:N2
            hold on
            plot3(sub_segment((j-dn2):j,1),sub_segment((j-dn2):j,2) ,sub_segment((j-dn2):j,3),...
                '.-', 'color', c(i,:), 'LineWidth',3);
            hold off
            axis([-lim,lim, -lim,lim, -lim,lim]) 
            
            pause(dt) 
            
       end
end
else
    figure(1)
    nrColors = 6;
    c = prism(nrColors);
    dp = floor(N/nrColors);
    for i =[1:nrColors]
        hold on
        plot3(globule((i-1)*dp+1:i*dp,1),globule((i-1)*dp+1:i*dp,2),...
            globule((i-1)*dp+1:i*dp,3),'.-', 'color', c(i,:), ...
            'LineWidth',3);
        hold off
    end
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
end
   
    axis(spans) 
    
    
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
if plot_traj
    axis equal;
hold on
plot3(traj(:,1),traj(:,2),traj(:,3),'.');
hold off
end

