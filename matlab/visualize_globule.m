

system('../_build/dna 100000');

data = load('../data/initial_state.m');
N = size(data,1);


lim = 2*floor(sqrt(N));
dt = 0.01;
dn = floor(N/1);
for i = (dn+1):dn:N
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3));
    hold on;
    plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3),'.');
    axis([-lim,lim, -lim,lim, -lim,lim]) 
    pause(0.01)
end

