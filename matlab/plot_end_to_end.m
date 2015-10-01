%A = load('../data/saved/meanSquareDistance.m');
A = load('../data/meanSquareDistance.m');
% Nr data points
N  = size(A(:,1),1);

n = A(:,1);
measures = A(:,2);
var_top = measures + A(:,3);
var_bot = measures - A(:,3);
theoretical = A(:,4);

mean_slope = ( log(measures(N)/measures(1)) ) / log(n(N) / n(1))
fitted = A(:,1).^mean_slope;
theoretical_slope = ( theoretical(N) - theoretical(1) ) / (n(N) - n(1))

figure(1)
l2 = loglog(n, var_top, 'xk');
hold on
for i= 1:N
   tmp = [var_top(i), var_bot(i)];
   tmp_n = [n(i), n(i)];
   loglog(tmp_n, tmp, 'k'); 
end

l3 = loglog(n, var_bot, 'xk');
l4 = loglog(n, measures, 'k');
l5 = loglog(n, theoretical);

title('Mean distance 100 samples per data-point');
xlabel('log Nr of links');
ylabel('log Mean distance');
legend([l4, l5], {'Measured data','theoretical'});

annotation('textbox', [.2 .8 .1 .1], 'String', ...
                    ['average slope of measured data is: ',num2str(mean_slope)]);