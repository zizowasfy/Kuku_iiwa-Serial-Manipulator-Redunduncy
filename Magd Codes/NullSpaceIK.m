function [all_configs, all_positions] = NullSpaceIK(current_config, trajectory, l, is_it_planar_trajectory, objective, jointsRange)

sp = 350;

current_p = FK(l, current_config, 0, '0 0 0', '1 0 0');
% final_p = FK(l, final_config, 0);

all_configs = {current_config};
all_positions = {current_p};

Robot = figure();

global axes_plot links_plot joints_plot end_effector_plot path_plot
axes_plot = plot3(0,0,0);
hold on
links_plot = plot3(0,0,0);
hold on
joints_plot = plot3(0,0,0);
hold on
end_effector_plot = plot3(0,0,0);
hold on 

nPoints = size(trajectory, 2);
for i=1:nPoints
    final_p = trajectory(:,i);
    
    if i == 2
        config_idx = length(all_configs);
    end
    while norm(final_p - current_p) > 50

        if ~ishandle(Robot), return, end
        delete([links_plot,joints_plot,end_effector_plot])
        delete(axes_plot)
        if i==1
                delete(path_plot)
        end

        dDistance = (final_p - current_p);
        velocity = sp * dDistance/norm(dDistance);

        config = cell2mat(all_configs(end));
        J = Jacobian(l, config);
        J_hash = pinv(J);

        dq0 = calc_dQ0(l, config, objective, jointsRange);

        dq = dq0 + J_hash*(velocity - J*dq0);
        new_config = config + dq;

        all_configs{end+1} = new_config;  

        current_p = FK(l, new_config, 1, '0 0 0', '1 0 0');
%         FK(l, [atan2d(current_p(2), current_p(1))+90 90 0 0 0 0 0], 1, "0 0 1", 'None');
        drawnow

        all_positions{end+1} = current_p;

    end 
end

all_configs = all_configs(config_idx:end);

delete([links_plot,joints_plot,end_effector_plot])
delete(axes_plot)
% for the planar shapes, it is better to set the y and x axes limits to be
% the same values, in order to visualize the drawn shape better.

% if is_it_planar_trajectory
%     daspect([1 1 1])
% end