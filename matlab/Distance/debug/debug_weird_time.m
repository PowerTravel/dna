clear all

addpath('..')

data = read_debug_data();

dt = data{1,2};
particle_radius = data{1,3};
particle_position = data{1,4};
particle_veclocity = data{1,5};

nr_coll_geoms = data{4,2}(1);
buggy_geom = data{4,2}(2)+1;
figure(1)

%% Plot particle
p = (data{1,4});
w = [data{1,3},data{1,3},data{1,3}];
[x,y,z]=ellipsoid(p(1),p(2),p(3),w(1),w(2),w(3),20);
hold on
surf(x, y, z,'FaceColor','g');
hold off
%% Plot Collision geoms

for i = 3:(nr_coll_geoms+2)
    p = (data{4,i}(3:2:7) + data{4,i}(2:2:6))./2;
    w = (data{4,i}(3:2:7) - data{4,i}(2:2:6))./2;
    [x,y,z]=ellipsoid(p(1),p(2),p(3),w(1),w(2),w(3),20);
    hold on
    if (i-2) == buggy_geom
    surf(x, y, z,'FaceColor','r');
    else 
    surf(x, y, z,'FaceColor','y');
    end
    hold off
end
grid on
axis equal