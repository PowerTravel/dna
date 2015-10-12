% This program checks if we have an increase in error as the number
% of links increases. fit1.a should be horizontal
B = load('../data/meanSquareDistance.m');

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