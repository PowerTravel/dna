function [ output_args ] = go_deep( level )
%GO_DEEP Summary of this function goes here
%   Detailed explanation goes here
    max_level = 3;
    max_index = 3;
    data = [-1,0,1];
    
    global string;
    
    if(level == max_level)
        string
        return
    end
    
    for i=1:3
        string(level)=data(i);
        go_deep(level+1)
    end


end

