
 
chain = importdata('R.dna');


na = chain(:,1);
R = chain(:,2);
R_var = chain(:,3);
%Rtheo = (0.9)*chain(:,4);
R_theo = chain(:,4);

Rg = chain(:,5);
Rg_var = chain(:,6);
Rg_theo = chain(:,7);
%Rg_theo = (2/5)*chain(:,7);
%% chain analyze points
steps = size(na,1);

f = fittype('a*x+b');
fitR_t = fit(log(na),log( R_theo ),f,'StartPoint',[1 1]);

figure(1)
%plot(fitR,'r-',log_n,log(R),'k.');
plot(log(na), log(R_theo), log(na), log(R) , '-k',...
                    log(na), log(R+sqrt(R_var)), '.k', log(na), log(R-sqrt(R_var)), '.k' )
title('loglog of end to end distance vs nr of links');
xlabel('Log of links');
ylabel('Log of end to end distance');
legend({'Theoretical', 'Data','Standard Deviation'});
annotation('textbox', [.2 .8 .1 .1], 'String', ...
                     ['Theoretical slope: ',num2str(fitR_t.a)]);

figure(2)
fitRg_t = fit(log(na),log( Rg_theo ),f,'StartPoint',[1 1]);
plot(log(na), log(Rg_theo), log(na), log(Rg), '-k',...
                    log(na), log(Rg+sqrt(Rg_var)), '.k', log(na), log(Rg-sqrt(Rg_var)), '.k')
title('loglog of Radius of gyration vs nr of links');
xlabel('Log of links');
ylabel('Log of radius of gyration');
legend({'Theoretical', 'Data','Standard Deviation'});
annotation('textbox', [.2 .8 .1 .1], 'String', ...
                     ['Theoretical slope: ',num2str(fitRg_t.a)]);

