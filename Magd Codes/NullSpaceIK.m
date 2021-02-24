function [all_configs, all_positions] = NullSpaceIK(current_config, final_config, nPoints, l, objective, jointsRange)

inv_sp = 1;

current_p = FK(l, current_config, 0);
final_p = FK(l, final_config, 0);

all_configs = {current_config};
all_positions = {current_p};

global axes_plot links_plot joints_plot end_effector_plot
axes_plot = plot3(0,0,0);
hold on
links_plot = plot3(0,0,0);
hold on
joints_plot = plot3(0,0,0);
hold on
end_effector_plot = plot3(0,0,0);

while norm(final_p - current_p) > 0.01
    
    delete([links_plot,joints_plot,end_effector_plot])
    delete(axes_plot)
    
    dDistance = (final_p - current_p);
    velocity = dDistance/(inv_sp*norm(dDistance));
    
    config = cell2mat(all_configs(end));
    J = Jacobian(l, config);
    J_hash = J'*inv(J*J');
    
    dq0 = calc_dQ0(l, config, objective, jointsRange);
    
    dq = J_hash*velocity + (eye(length(config)) - J_hash*J)*dq0;
    new_config = config + dq;
    
    all_configs{end+1} = new_config;  
    
    current_p = FK(l, new_config, 1);
    FK(l, final_config, 1);
    drawnow
    
    all_positions{end+1} = current_p;
%     pause(0.2)
    view(0,90)
end