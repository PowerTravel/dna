A = load('../data/default_data.dna');
%A = load('../data/1000_measures_end_to_end_data.dna')
% Nr data points
N  = size(A(:,1),1);

n = A(:,1);
measures = A(:,2);
var_top = measures + A(:,3);
var_bot = measures - A(:,3);
theoretical = A(:,4);

theoretical_slope = log(theoretical(N)/theoretical(1))  / log(n(N)/ n(1))



ydata  = log(measures);
xdata = log(n);
f = fittype('a*x+b');
fit1 = fit(xdata,ydata,f,'StartPoint',[1 1]);
figure(1);
plot(fit1,'r-',xdata,ydata,'k.')

fit1
mean_slope = fit1.a;
fitted = A(:,1).^mean_slope;

figure(2)
%l2 = loglog(n, var_top, 'xk');
%
%for i= 1:N
%   tmp = [var_top(i), var_bot(i)];
%   tmp_n = [n(i), n(i)];
%   loglog(tmp_n, tmp, 'k'); 
%end

%l3 = loglog(n, var_bot, 'xk');
l4 = loglog(n, measures, 'k');
hold on
l5 = loglog(n, theoretical);
l6 = loglog(n, fitted, 'g');

title('Mean distance 100 samples per data-point');
xlabel('log Nr of links');
ylabel('log Mean distance');
legend([l4, l6, l5], {'Measured data', 'Fitted slope','theoretical'});

annotation('textbox', [.2 .8 .1 .1], 'String', ...
                    ['average slope of measured data is: ',num2str(mean_slope)]);