
for i=1:1
%cd '../../../_build'
%system('./dna')
%cd '../matlab/Distance/debug'
 
data = load('trajectory');
%data = load('../particle_trajectory.dna');

N = size(data,1);
animate = 1;
x = 1:N;

sep_x = 0.9;
sep_y = 2;

%plot_epsiloid([sep_x, -sep_y, -sep_x], [1,1,1]);
%plot_epsiloid([-sep_x, -sep_y, -sep_x], [1,1,1]);
%plot_epsiloid([sep_x, -sep_y, sep_x], [1,1,1]);
%plot_epsiloid([-sep_x, -sep_y, sep_x], [1,1,1]);

%plot_epsiloid([sep_x, sep_y, -sep_x], [1,1,1]);
%plot_epsiloid([-sep_x, sep_y, -sep_x], [1,1,1]);
%plot_epsiloid([sep_x, sep_y, sep_x], [1,1,1]);
%plot_epsiloid([-sep_x, sep_y, sep_x], [1,1,1]);
        

frame_jump = 1;
if(animate==1)
    plot3(data(:,1),data(:,2),data(:,3),'.');
    hold on
    xlabel('x')
    ylabel('y')
else
    r=5;
    xlabel('x')
    ylabel('y')
    zlabel('z')
    for i = 2:N
        if(mod(i,1)==0)
            hold on
            if (abs(data(i,1) - data(i-1,1)) < 1) & ...
                (abs(data(i,2) - data(i-1,2)) < 1) & ... 
                (abs(data(i,3) - data(i-1,3)) < 1) 
                plot3(data(i-1:i,1),data(i-1:i,2), data(i-1:i,3), 'linewidth', 1);
            end
            axis([-r,r,-r,r,-r,r]);
            %pause(0.1);
        end
    end
    hold off
end
pause(0.1)
end
