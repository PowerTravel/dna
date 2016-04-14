hold off
clear all
DData = load('distance_3.dna');

tmpt1 = DData(1,1);
tmpt2 = DData(2,1);
i=3;
while( (tmpt1<tmpt2) && (i<=size(DData,1)))
    tmpt1 = tmpt2;
    tmpt2 = DData(i,1);
    i = i+1;
end

N = i-1

NrN = size(DData,1)/N


slopes = zeros(NrN,1);
for LOL = 1:NrN

    RunIdx = LOL-1;    
    NStart = 1+RunIdx*N;
    Nend = NStart + N-1;
    
%N = size(DData,1);

time = DData(NStart:Nend,1);
lgt = log(time);

D = DData(NStart:Nend,2);
D_v = DData(NStart:Nend,3);
P = DData(NStart:Nend,4:6);
P_v = DData(NStart:Nend,7:9);

f = fittype('a*x+b');
fitD_m = fit(lgt,log( D ),f,'StartPoint',[1 1]);
slopes(LOL) = fitD_m.a;
figure(1);
plot(lgt, log(D), '-k.', lgt ,log(D+sqrt(D_v)), ...
            '.k', lgt, log(D-sqrt(D_v)), '.k', 'linewidth',2);
   
title('loglog of diffusion distance vs time');
xlabel('Log of time');
ylabel('Log diffusion distance');
legend({'Data','Standard Deviation'});
annotation('textbox', [.2 .8 .1 .1], 'String', ...
                     ['Measured slope: ', num2str(fitD_m.a) ]);
                
figure(2)
plot3(P(:,1),P(:,2),P(:,3))
end

%plot([0.01,0.0575,0.105,0.1525,0.2], slopes);