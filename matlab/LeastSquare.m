clear all

A = load('../data/meanSquareDistance.m');

ydata  = A(:,5);
xdata = log(A(:,1));

f = fittype('a*x+b');
fit1 = fit(xdata,ydata,f,'StartPoint',[1 1]);


figure(1);
plot(fit1,'r-',xdata,ydata,'k.')

figure(2);
plot(xdata, ydata)

fit1.a