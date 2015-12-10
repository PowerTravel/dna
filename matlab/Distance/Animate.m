data = load('../../data/default_distance.dna');

N = size(data,1)


plot3(data(:,1),data(:,2),data(:,3),'.','markersize',15)
%for i = 1:N
%    plot3(data(i,1:3))
%    pause
%end

