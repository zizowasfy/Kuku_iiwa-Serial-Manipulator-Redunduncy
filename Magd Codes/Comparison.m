function [all_configs1, all_positions1] = Comparison(current_config, trajectory, l, is_it_planar_trajectory, objective, jointsRange)

sp = 500;

current_p1 = FK(l, current_config, 0, "0 0 0");
current_p2 = FK(l, current_config, 0, "0 0 0");
% final_p = FK(l, final_config, 0);

all_configs1 = {current_config};
all_positions1 = {current_p1};

all_configs2 = {current_config};
all_positions2 = {current_p1};

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

W = diag([0.5, 0.5, 0.5, 0.8, 0.8, 0.8, 1]);
nPoints = size(trajectory, 2);
for i=1:nPoints
    final_p = trajectory(:,i);
    while  (norm(final_p - current_p1) > 10) | (norm(final_p - current_p1) > 10)

        if ~ishandle(Robot), return, end
        delete([links_plot,joints_plot,end_effector_plot])
        delete(axes_plot)
        if i==1
                delete(path_plot)
        end
        
        % ###### Method 1 (Pseudoinverse) ######:
        dDistance1 = (final_p - current_p1);
        velocity1 = sp * dDistance1/norm(dDistance1);

        config1 = cell2mat(all_configs1(end));
        J1 = Jacobian(l, config1);
        J_hash1 = pinv(J1);

        dq1 = J_hash1*velocity1;
        new_config1 = config1 + dq1;

        all_configs1{end+1} = new_config1;  

        current_p1 = FK(l, new_config1, 1, '1 0 0', '1 1 0');
%         FK(l, final_config, 1);
%         drawnow

        all_positions1{end+1} = current_p1;
        
        % ###### Method 2 (Null Space)######:
        dDistance2 = (final_p - current_p2);
        velocity2 = sp * dDistance2/norm(dDistance2);

        config2 = cell2mat(all_configs2(end));
        J2 = Jacobian(l, config2);
        J_hash2 = W'*J2' * pinv(J2*W'*J2');

        dq0_2 = calc_dQ0(l, config2, objective, jointsRange);

        dq2 = dq0_2 + J_hash2*(velocity2 - J2*dq0_2);
        new_config2 = config2 + dq2;

        all_configs2{end+1} = new_config2;  

        current_p2 = FK(l, new_config2, 1, '0 0 1', '1 0 1');
        %         FK(l, final_config, 1);
        drawnow

        all_positions2{end+1} = current_p2;

    end 
end

% for the planar shapes, it is better to set the y and x axes limits to be
% the same values, in order to visualize the drawn shape better.

if is_it_planar_trajectory
    daspect([1 1 1])
end