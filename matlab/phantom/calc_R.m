function [na , R, R_var ] = calc_R( chain, start_point, steps, expo )

samples = size(chain,2)/3;
N = size(chain,1);

n = zeros(steps+1,1);
R_tmp = zeros(steps, samples);
R_var = zeros(steps, 1);
 
for i = [1:steps+1]
    if(expo)
        k = log( N / start_point ) / (steps);
        n(i) = floor(start_point*exp(k*(i-1)));
    else
        dn = (N-start_point)/(steps);
        n(i) = floor((i-1)*dn+start_point);
    end
end
na = (n(1:steps) + n(2:steps+1))/2;

for s = [1:samples]
    
    %% Bin values
    for i = [1:steps]
        bin_t = chain(n(i):n(i+1),3*s-2:3*s);
        if size(bin_t,1) > 1
            tmp = mean(bin_t);
        else
            tmp = bin_t;
        end
        R_tmp(i,s) = R_tmp(i,s) + norm(tmp,2);
        
    end

end

R = mean(R_tmp,2);
size(R_tmp(1,:))
for s = 1:steps
    R_var(s) = sum(( R_tmp(s,:) - R(s) ).^2);
end
R_var = R_var ./ samples;
samples

end

