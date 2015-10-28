
 
chain = importdata('R.dna');

na = chain(:,1);
R = chain(:,2);
Rvar = chain(:,3);
%Rtheo = (0.9)*chain(:,4);
Rtheo = chain(:,4);

Rg = chain(:,5);
Rg_var = chain(:,6);
Rg_theo = chain(:,7);
%Rg_theo = (2/5)*chain(:,7);
%% chain analyze points
steps = size(na,1);



figure(1)
%plot(na, theo_chain, na, R, na, R+sqrt(Rvar), na, R-sqrt(Rvar))
plot(log(na), log(Rtheo), log(na), log(R),...
                    log(na), log(R+sqrt(Rvar)), log(na), log(R-sqrt(Rvar)) )
figure(2)

plot(log(na), log(Rg_theo), log(na), log(Rg),...
                    log(na), log(Rg+sqrt(Rg_var)), log(na), log(Rg-sqrt(Rg_var)))

%f = fittype('a*x+b');
%fit1 = fit(log(na),log( R ),f,'StartPoint',[1 1])

