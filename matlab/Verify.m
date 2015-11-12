%system('../_build/dna')

A = importdata('../matlab/phantom/R.dna');
%A = importdata('../data/Verify_Default.dna');
%A = importdata('../data/saved/Verify_N40000_Samples1000.dna');

N  = size(A(:,1),1);
n = A(:,1);

log_n = log(n);

%% R
R = A(:,2);
R_var = A(:,3);
R_theo = A(:,4);
%R_err = A(:,5);

f = fittype('a*x+b');
w = 1./(sqrt(R_var./1000));
w(1) = 0;
w = w./sum(w);
%fit1 = fit(log_n,log( R ),f,'StartPoint',[1 1], 'weight', w);
fit1 = fit(log_n,log( R ),f,'StartPoint',[1 1]);
fit1_t = fit(log_n,log( R_theo ),f,'StartPoint',[1 1]);
%fit1_err = fit(n,R,f,'StartPoint',[1 1]);
%R_Error_increase = fit1_err.a

figure(1);

R_mean_slope = fit1.a;
plot(fit1,'r-',log_n,log(R),'k.');
hold on
plot(log_n, log(R_theo));

title('loglog of end to end distance vs nr of links');
xlabel('Log of links');
ylabel('Log of end to end distance');
legend({'Measured data', 'Fitted slope','theoretical'}, 'Location' ,'Best');
cnf = confint(fit1, 0.95);
cnf = cnf(2,1) - cnf(1,1);
annotation('textbox', [.2 .8 .1 .1], 'String', ...
                     ['average slope of measured data is: ',num2str(R_mean_slope), ...
                       ' +- ', num2str(cnf,1), char(10) ...
                       'with 95% confidence interval. Theoretical slope is ', num2str(fit1_t.a) ]);

% The slope should be 0, that is error should not increase with # links
%R_error = fit(log_n, log(R_err), f, 'StartPoint',[1 1]);
%R_error_slope = R_error.a;

%figure(2)
%hist(R_var);
%title('Variance of end to end distance vs nr of links');
%xlabel('# of links');
% R_gyr
Rg = A(:,5);
Rg_var = A(:,6);
Rg_theo =1/sqrt(6)  * A(:,7);
Rg_theo =A(:,7);
%Rg_err = A(:,9);

f = fittype('a*x+b');
fit2 = fit(log_n,log( Rg ),f,'StartPoint',[1 1]);
fit2_t = fit(log_n,log( Rg_theo ),f,'StartPoint',[1 1]);
%fit2_err = fit(n,Rg_err,f,'StartPoint',[1 1]);
%R_Gyr_Error_increase = fit2_err.a


figure(3);

Rg_mean_slope = fit2.a;
plot(fit2,'r-',log_n,log(Rg),'k.');
hold on
plot(log_n, log(Rg_theo));
title('loglog of Radius of gyration vs nr of links');
xlabel('Log of links');
ylabel('Log of radius of gyration');
legend({'Measured data', 'Fitted slope','theoretical'}, 'Location' ,'Best');
cnf = confint(fit2, 0.95);
cnf = cnf(2,1) - cnf(1,1);
annotation('textbox', [.2 .8 .1 .1], 'String', ...
                     ['average slope of measured data is: ',num2str(Rg_mean_slope), ...
                       ' +- ', num2str(cnf,1), char(10) ...
                       'with 95% confidence interval. Theoretical slope is ', num2str(fit2_t.a) ]);
%figure(4)
%hist(Rg_var);
%title('Variance of Radius of gyration vs nr of links');
%xlabel('# of links');
% %% CM
% cm = A(:,10);
% cm_var = A(:,13);
% cm_theo = A(:,16);
% cm_err = A(:,19);
% 
% f = fittype('a*x+b');
% fit3 = fit(n ,cm ,f,'StartPoint',[1 1]);
% fit3_t = fit(n ,cm_theo ,f,'StartPoint',[1 1]);
% fit3_err = fit(n,cm,f,'StartPoint',[1 1]);
% CM_Error_increase = fit3_err.a
% 
% figure(5);
% 
% cm_mean_slope = fit3.a;
% plot(fit3,'r-',n,cm,'k.');
% hold on
% plot(n, cm_theo);
% title('Center of mass vs nr of links');
% xlabel('Links');
% ylabel('Center of mass');
% legend({'Measured data', 'Fitted slope','theoretical'}, 'Location' ,'Best');
% cnf = confint(fit3, 0.95);
% cnf = cnf(2,1) - cnf(1,1);
% annotation('textbox', [.2 .8 .1 .1], 'String', ...
%                      ['average slope of measured data is: ',num2str(cm_mean_slope), ...
%                        ' +- ', num2str(cnf,1), char(10) ...
%                        'with 95% confidence interval. Theoretical slope is ', num2str(fit3_t.a) ]);
% % figure(6)
% % hist(cm_var);
% %title('Variance in Center of mass vs nr of links');
% %xlabel('# of links');