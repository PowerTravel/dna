data = load('../../data/default_distance.dna');

N = size(data,1);
animate = false;

r = 5;
axis([-r,r,-r,r,-r,r]);
if(~animate)
    plot(data(:,1),data(:,2),'.','markersize',15)
else
    for i = 1:N
        plot(data(i,1),data(i,2),'.','markersize',15);
        hold on
        pause(0.001);
    end
    hold off
end
