function [ chain ] = gen_phantom( N, samples )
%GEN_PHANTOM Summary of this function goes here
%   Detailed explanation goes here

%% Generate chain
chain = zeros(N, 3*samples);
DIM = 6;

for s = [1:samples]
    
    for i = [2:N]
        a = floor(random('unif',1,DIM+1));

        new_pos = getIdx(a);
        chain(i,3*s-2:3*s) = chain(i-1,3*s-2:3*s) + new_pos;
    end
    
end

end

function [R] = getIdx(i)
	switch i 
        case 1
	        a = 1;
            b = 0;
            c = 0;
	    case 2
            a = -1;
            b = 0;
            c = 0;
	    case 3
            a = 0;
            b = 1;
	        c = 0;
        case 4    
	        a = 0;
            b = -1;
            c = 0;
	    case 5
            a = 0;
            b = 0;
	        c = 1;
        case 6
            a = 0;
            b = 0;
	        c = -1;
	end
    R = [a,b,c];
end