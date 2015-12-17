data = load('../../data/default_distance.dna');

N = size(data,1);
animate = false;
x = 1:N;
figure(1)
plot(x, data(:,9),x, data(:,8),x, data(:,7));
legend({'e_tot', 'Ep', 'Ek'})

f = @(t,r,x,y) sqrt(r.^2-(t-x).^2)+y;

figure(2)
frame_jump = 5;
if(~animate)
    %plot(data(:,1),data(:,2),'.','markersize',15)
    plot(data(:,1),data(:,2),'.');
    hold on
    T = 0:0.1:20;
    plot(T , f(T,20,5,-22))
    %plot(T , f(T,20,-5,-22))
else
    r = 5;
    
    for i = 1:N
        if(mod(i,frame_jump) == 0)
        plot(data(i,1),data(i,2),'.','markersize',15);
        axis([-r,r,-r,r]);
        pause(0.001);
        end
    end
    hold off
end
