% Author ~ Ahmed Magd Aly
% Innopolis University

function pose = FK(l, q, enable_plot)
% Forward Kinematics Function:

q = deg2rad(q);

%% Origin Frame
O0 = eye(4); % world reference frame

%% Moving to point 1:

% FK = Rz(q(1)) * Tx(l(1)) * Rz(q(2)) * Tx(l(2)) * Rz(q(3)) * Tx(l(3)) * Rz(q(4)) * Tx(l(4));

O1 = eye(4);

T1 = Rz(q(1)) * Tx(l(1));

O2 = O1*T1;

T2 = Rz(q(2)) * Tx(l(2));

O3 = O2*T2;

T3 = Rz(q(3)) * Tx(l(3));

O4 = O3*T3;

T4 = Rz(q(4)) * Tx(l(4));

O5 = O4*T4;

O = round([O0, O1, O2, O3, O4, O5],10);

global axes_plot links_plot joints_plot end_effector_plot



if enable_plot
    % figure('units','normalized','outerposition',[0 0 1 1])
    


    as = 1; % axes scaler
    color = ['r','g','b']; % axes color
    
    index = 0;
    for i = 1:4:length(O)
        index = index + 1;
        points_x(index) = O(1,i+3);
        points_y(index) = O(2,i+3);
        points_z(index) = O(3,i+3);
        
        if index ~= 2 && index ~= 5
            for axes = 0:2

                axes_plot = [axes_plot plot3([O(1,i+3) as*O(1,i+axes)+O(1,i+3)], [O(2,i+3) as*O(2,i+axes)+O(2,i+3)], [O(3,i+3) as*O(3,i+axes)+O(3,i+3)],'Color',color(axes+1))];
                hold on
            end
        end
    end
    joints_x = [points_x(1:end)];
    joints_y = [points_y(1:end)];
    joints_z = [points_z(1:end)];
    
    links_plot = [links_plot plot3(points_x, points_y, points_z,'Color', "0 0 0",'linewidth',2)];
    hold on
    joints_plot = [joints_plot plot3(joints_x(1:5), joints_y(1:5), joints_z(1:5),'.','Color','0.992 0.788 0.04 1','MarkerSize',20)];
    hold on
    end_effector_plot = [end_effector_plot plot3(joints_x(end), joints_y(end), joints_z(end),'.','Color','0.8 0 0 1','MarkerSize',20)];
    path_plot = plot3(joints_x(end), joints_y(end), joints_z(end),'.','Color','0.8 0 0 1','MarkerSize',7.5);
    
    
%     xlim([-5 5])
%     ylim([-5 5])
%     zlim([-5 5])
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
%     view(0,0)
    grid on
end

angles = rotm2eul(O5(1:3, 1:3))';
pose = [O5(1:2, 4); angles(3)];

