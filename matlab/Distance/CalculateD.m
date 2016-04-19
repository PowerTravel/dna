% c is time dependant and must be scaled against trivial case.;
% ct holds the timestep where it was measured
ct = [0.01, 0.05, 0.1, 0.2, 0.4, 0.8, 1.6]';
cv = [3.0169e+04, 1.2026e+03, 302.2110,74.1227, 18.7605, 4.6821, 1.1598]';


f = fittype('a*x+b');

fitC = fit(log(ct),log( cv ),f,'StartPoint',[1 1]);
fitC.a
cxp = ct.^(-2);
cmean = mean(cv./cxp)
cxp = cxp*cmean;

plot(log(ct), log(cxp), log(ct), log(cv))
