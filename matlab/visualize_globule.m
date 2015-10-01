
gen = false;
nrSims = 100;



if gen
len = zeros(nrSims,1);

for i = 1:nrSims

system('../_build/dna 100');

data = load('globule_data.m');

N = size(data,1);

len(i) = norm(data(1,:) - data(N,:),2);
len(i)
pause(1)

end

end

norm(len)
var(len)

% dt = 0.01;
% dn = floor(N/100);
% for i = (dn+1):dn:N
%     plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3));
%     hold on;
%     plot3(data((i-dn):i,1),data((i-dn):i,2),data((i-dn):i,3),'.');
%     axis([-lim,lim, -lim,lim, -lim,lim]) 
%     pause(0.01)
% end

