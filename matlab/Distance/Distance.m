hold off
clear all

%DData = load('distance_1.dna');Sim10000Globules_data.dna
DData = load('Sim10000Globules_data.dna');
f = fittype('a*x+b');

tmpt1 = DData(1,1);
tmpt2 = DData(2,1);
i=3;
while( (tmpt1<tmpt2) && (i<=size(DData,1)))
    tmpt1 = tmpt2;
    tmpt2 = DData(i,1);
    i = i+1;
end

N = i-1

if( N ~= size(DData,1))
    N = i-2
end
NrN = size(DData,1)/N

slopes = zeros(NrN,1);
for SimulationRun = 1:NrN
    SimulationRun
    RunIdx = SimulationRun-1;
    NStart = 1+RunIdx*N;
    Nend = NStart + N-1;

%N = size(DData,1);

time = DData(NStart:Nend,1);

theo = @(t,d,D) 2*d*D.*t;
Dt = theo(time,3,1);
lDt = log(Dt);
lgt = log(time);

D = DData(NStart:Nend,2);
D_v = DData(NStart:Nend,3);
P = DData(NStart:Nend,4:6);
P_v = DData(NStart:Nend,7:9);

Binned_Data = DData(NStart:Nend, 10:end);
NrOfSamples = size(Binned_Data, 2);

cv = Dt./D;
c = mean(cv);

fitD_m = fit(lgt,log( D ),f,'StartPoint',[1 1]);
slopes(SimulationRun) = fitD_m.a;
figure(1);
hold on
plot(lgt, log(D), '-k.', lgt ,log(D+sqrt(D_v)), ...
            '.k', lgt, log(D-sqrt(D_v)), '.k', 'linewidth',2);
   
title('loglog of <x^2> vs time');
xlabel('Log of time');
ylabel('Log <x^2>');
legend({'Data','Standard Deviation'});
annotation('textbox', [.2 .8 .1 .1], 'String', ...
                     ['Measured slope: ', num2str(fitD_m.a) ]);

                 
%for sample = 1:NrOfSamples

    x = Binned_Data(end,1:3:end-2);
    y = Binned_Data(end,2:3:end-1);
    z = Binned_Data(end,3:3:end);
    
    nr_bins = 100;
    dx = [ (max(x) - min(x)), max(y)-min(y), max(z) - min(z) ] ./nr_bins;
    
    %histogram(x,'Normalization', 'probability')
   
    P = [ hist(x,nr_bins); hist(y,nr_bins); hist(z,nr_bins)];
    P(1,:) = P(1,:)./( sum(P(1,:)).*dx(1));
    P(2,:) = P(2,:)./( sum(P(2,:)).*dx(2));
    P(3,:) = P(3,:)./( sum(P(3,:)).*dx(3));
    
    pedf = @(x,t,D)(1./sqrt(4*pi*D.*t)).*exp(-(x.*x)./(4*D.*t));
    
    sx = min(x)+dx(1)/2:dx(1):max(x);%-dx(1)/2;
    sy = min(y)+dx(2)/2:dx(2):max(y);%-dx(2)/2;
    sz = min(z)+dx(3)/2:dx(3):max(z);%-dx(3)/2;
    
    xx = min(x):0.01:max(x);
    xy = min(y):0.01:max(y);
    xz = min(z):0.01:max(z);
    
    figure(3*SimulationRun-1)
    hold on
    bar(sx,P(1,:))
    plot(xx,pedf(xx,time(end),1), 'linewidth', 2)
    title(['X Simulation ', num2str(SimulationRun)])
    xlabel('X')
    ylabel('Px')
    
    figure(3*SimulationRun)
    hold on
    bar(sy, P(2,:))
    plot(xy,pedf(xy,time(end),1), 'linewidth', 2)
    title(['Y Simulation ', num2str(SimulationRun)])
    xlabel('Y')
    ylabel('Py')
    figure(3*SimulationRun+1)
    hold on
    bar(sz, P(3,:))
    plot(xz,pedf(xz,time(end),1), 'linewidth', 2)
    title(['Z Simulation ', num2str(SimulationRun)])
    xlabel('Z')
    ylabel('Pz')

    
%end

end



figure(1)
hold on
plot(lgt, lDt);
hold off
%plot([0.01,0.0575,0.105,0.1525,0.2], slopes);