
function yyy = plot_expanded( mapname, states);
VoxelizedMap(mapname);
hold on;

if (1==mode)
    states=strcat(states,'_exp.txt');
elseif (2==mode)
    states=strcat(states,'_sol.txt');
end

debug_state = load(states);

x = debug_state(:,1);
y = debug_state(:,2);
z = debug_state(:,3);
yaw = debug_state(:, 4);


% plot mode: 1 plot with direction /  2 plot heatmap
plotmode = 1;

if 1 == plotmode 
   
    a = 0;
    if(0 == a )   % plot at once

        plot3(x, y, z, 'r*');

    else 
        pause on;
        num = size(x);
         for i = 1:num
            plot3(x(i), y(i), z(i), 'o', 'r');
            %pause(0.25);
            hold on;
        end
    end

    grid on;

else                    % plot density
    
    scatplot(x,y,'circle', 5, 50, 5, 1, 2);
    colorbar;
    
end

axis equal;
xlim([0 10]);
ylim([0 10]);
zlim([0 2]);
xlabel('x');
ylabel('y');
