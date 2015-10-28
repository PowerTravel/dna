
    
clear all
gen_chain = false;
pplot = false;  % plot globule


if(gen_chain)
    N = 5000;
    samples = 100;
    dlmwrite('chain.data', gen_phantom(N,samples) ); 
end

chain = importdata('chain.data');

%% chain analyze points
start_point = 10;
steps =50;
expo = true;   % use exponential steps

[na, R, R_var] = calc_R(chain, start_point, steps, expo);
%Rvar = zeros(steps, samples);



theo_chain = na.^(1/2);
%theo_chain = na.^(0.588);


figure(1)
plot(na, theo_chain, na, R, na, R+sqrt(R_var), na, R-sqrt(R_var))


plot(log(na), log(theo_chain), log(na), log(R),...
                    log(na), log(R+sqrt(R_var)), log(na), log(R-sqrt(R_var)))

f = fittype('a*x+b');
fit1 = fit(log(na),log( R ),f,'StartPoint',[1 1])


if pplot
    figure(2)
    for s = [1:size(chain,2)/3]
        plot3( chain(:,3*s-2), chain(:,3*s-1), chain(:,3*s),'.-')
        hold on
    end
    [x ,y, z] = sphere();
    x =x.*theo_chain(size(theo_chain,1));
    y =y.*theo_chain(size(theo_chain,1));
    z =z.*theo_chain(size(theo_chain,1));
    plot3(x,y,z)
end


