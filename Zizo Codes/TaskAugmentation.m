function TaskAugmentation(current_config, l, xsq, ysq, zsq, n)

    sp = 250;
    jn = 7;     % number of joints in the Serial manipulator            % Change Here
    
    current = FK(l, current_config, 0, "0 0 0", "1 0 0");
    q = current_config;
    
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
    
    for p = 1:4*n

        final = [xsq(p); ysq(p); zsq(p)];
        i = 1;
        while (sum(abs(final - current(1:3))) > 10)
            
            if ~ishandle(Robot), return, end
            delete([links_plot,joints_plot,end_effector_plot])
            delete(axes_plot)
            
            if p==1
                delete(path_plot)
            end
            r1 = (final - current(1:3))/1;  % Change Here
            r1 = sp * r1/norm(r1);
            J = Jacobian(l, q);   % Change Here
            J1 = J(1:3,:);
            J_hash = J1' * inv(J1*J1');
            J2 = J(4:5,:);%Jacobian(6,:);   % Change Here  
            r2 = [0; 0];              % sum of joints = 0 ( Task 2 )         % Change Here
            Joint_step = J_hash * r1 + pinv((J2*(eye(jn) - J_hash*J1))) * (r2 - J2*J_hash*r1);
            qs(1:jn,i) = q + Joint_step;

%         % visualizing the Robot's Path
%             clf
%             robot(L,qs(:,i));
%             pause(0.01)

        % updating the variables  
            q = qs(1:jn,i);
%             FK = localFK(q,L);
%             Jacobian = Jacob(eye(4), FK, q, L);
            current = FK(l, q, 1, "0 0 0", '1 0 0');
%             FK(l, [atan2d(current(2), current(1))+90 90 0 0 0 0 0], 1, "0 0 1", 'None');
            i = i + 1;
            xlim([-10 600])
            ylim([-10 600])
%             ylim([-10 400])
            view(23, 37)
            drawnow

        end
        if p == 1 | p == 2*n | p == 3*n | p == 4*n
            pause(5)
    end
    end

end