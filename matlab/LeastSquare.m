% This program checks if we have an increase in error as the number
% of links increases. fit1.a should be horizontal
B = load('../data/meanSquareDistance.m');

ydata  = B(:,5);
xdata = log(B(:,1));

f = fittype('a*x+b');
fit1 = fit(xdata,ydata,f,'StartPoint',[1 1]);


figure(1);
plot(fit1,'r-',xdata,ydata,'k.')

figure(2);
plot(xdata, ydata)

fit1.a