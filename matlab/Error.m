% This program checks if we have an increase in error as the number
% of links increases. fit1.a should be horizontal

B = load('../data/default_data.dna');
%B = load('../data/1000_measures_end_to_end_data.dna')

logerr  = B(:,5);
loglinks = log(B(:,1));
logdata  = log(B(:,2));


f = fittype('a*x+b');
error = fit(loglinks,logerr,f,'StartPoint',[1 1]);
slope = fit(loglinks,logdata,f,'StartPoint',[1 1]);


figure(1);
plot(error,'r-',loglinks,logerr,'k.')

figure(2);
plot(slope,'r-',loglinks,logdata,'k.')

error
slope