function [all_configs, all_positions] = PseudoInverseIK(current_config, trajectory, l, is_it_planar_trajectory, W)

sp = 300;

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
    counter = 0;
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
        J_hash = inv(W)* J' * inv(J*inv(W)*J');
        dq = J_hash*velocity;
        new_config = config + dq;

        all_configs{end+1} = new_config;  

        current_p = FK(l, new_config, 1, '0 0 0', '1 0 0');
%         FK(l, final_config, 1 , '0 0 1', 'None');
        drawnow

        all_positions{end+1} = current_p;
    %     pause(0.2)
        counter = counter + 1;
        if counter > 25
            break
        end
    end    
end

all_configs = all_configs(config_idx:end);

delete([links_plot,joints_plot,end_effector_plot])
delete(axes_plot)