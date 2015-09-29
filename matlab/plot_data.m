system('../_build/dna 1000')

data = load('globule_data.m');

N = size(data,1);
dt = 0.01;

i=N;
lim = 40;
dn = floor(N/100);
if dn == 0
   dn = 1; 
end

for i = 1:dn:N
    plot3(data(1:i,1),data(1:i,2),data(1:i,3));
    hold on;
    plot3(data(1:i,1),data(1:i,2),data(1:i,3),'.');
    axis([-lim,lim, -lim,lim, -lim,lim]) 
    pause(0.01)
end