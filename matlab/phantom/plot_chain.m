function [ ] = plot_chain( chain )

samples = size(chain,2)/3;

for s = [1:samples]
    plot3(chain(:,3*s-2),chain(:,3*s-1),chain(:,3*s),'.-')
    hold on
end
hold off
end

