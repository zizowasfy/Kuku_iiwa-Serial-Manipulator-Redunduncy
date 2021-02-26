function [all_configs, all_positions] = PseudoInverseIK(current_config, final_config, l)

inv_sp = 0.001;
current_p = FK(l, current_config, 0);
final_p = FK(l, final_config, 0);

all_configs = {current_config};
all_positions = {current_p};

Robot = figure();

global axes_plot links_plot joints_plot end_effector_plot
axes_plot = plot3(0,0,0);
hold on
links_plot = plot3(0,0,0);
hold on
joints_plot = plot3(0,0,0);
hold on
end_effector_plot = plot3(0,0,0);
hold on

while norm(final_p - current_p) > 0.001
    
    if ~ishandle(Robot), return, end
    delete([links_plot,joints_plot,end_effector_plot])
    delete(axes_plot)
    dDistance = (final_p - current_p);

    velocity = dDistance/0.5;

    config = cell2mat(all_configs(end));
    J = Jacobian(l, config);
    J_hash = pinv(J);
    dq = J_hash*velocity;
    new_config = config + dq;

    all_configs{end+1} = new_config;  
    
    current_p = FK(l, new_config, 1);
    FK(l, final_config, 1);
    drawnow
    
    all_positions{end+1} = current_p;
%     pause(0.2)
end