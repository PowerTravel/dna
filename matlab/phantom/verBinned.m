
 
chain = importdata('binned_chain.data');

na = chain(:,1);
chain = chain(:,2:end);


%% chain analyze points
steps = size(chain,1);
samples = size(chain,2)/3;

Rtmp = zeros(steps,samples);
Rvar = zeros(steps,1);
for i = 1:steps
   
    for j = 1:samples
       Rtmp(i,j) = norm(chain(i, 3*j-2:3*j ));
    end    
   
end

R = mean(Rtmp,2);

for s = 1:steps
    Rvar(s) = sum((Rtmp(s,:) - R(s)).^2) ./ samples;
end
%Rvar = zeros(steps, samples);

theo_chain = na.^(1/2);
%theo_chain = na.^(0.588);


%figure(1)
%plot(na, theo_chain, na, R, na, R+sqrt(Rvar), na, R-sqrt(Rvar))
plot(log(na), log(theo_chain), log(na), log(R),...
                    log(na), log(R+sqrt(Rvar)), log(na), log(R-sqrt(Rvar)) )


%plot(log(na), log(theo_chain), log(na), log(R),...
%                    log(na), log(R+sqrt(R_var)), log(na), log(R-sqrt(R_var)))

%f = fittype('a*x+b');
%fit1 = fit(log(na),log( R ),f,'StartPoint',[1 1])

