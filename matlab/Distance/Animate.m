data = load('../../data/default_distance.dna');

N = size(data,1);
animate = false;
x = 1:N;
%figure(1)
%plot(x, data(:,9),x, data(:,8),x, data(:,7));
%legend({'e_tot', 'Ep', 'Ek'})

f = @(t,r,x,y) sqrt(r.^2-(t-x).^2)+y;

plane = @(p,k, x) k*(p(1)-x) + p(2);
hold on
%figure(2)
frame_jump = 1;
if(~animate)
    T = -10:0.1:10;
    %plot(data(:,1),data(:,2),'.','markersize',15)
    plot3(data(:,1),data(:,2),data(:,3),'.');
    hold on
    T = -10:0.1:10;
    %plot(T, plane([0,-3],-1,T) );
    %plot(T, plane([0,-3],1,T) );
    xlabel('x')
    ylabel('y')
    %plot(T , f(T,20,5,-22))
    %plot(T , f(T,20,-5,-22))
else
    r = 5;
    
    for i = 1:N
        if isnan( [data(i,1)] ) == 1
           i
           break
        end
        if(mod(i,frame_jump) == 0)
        A = data(i,1)-0.4999:0.01:data(i,1)+0.4999;
        %plot(data(i,1),data(i,2),'.','markersize',15);
        plot(A,f(A,0.5,data(i,1),data(i,2)));
        hold on
        plot(A,-f(A,0.5,data(i,1),-data(i,2)));
        T = -10:0.1:10;
     %   plot(T, plane([0,-3],-1,T) );
     %   plot(T, plane([0,-3],1,T) );
        plot(T, plane([0,0],0,T) );
        axis([-r,r,-r,r]);
        pause(0.001);
        hold off
        end
    end
    hold off
end
