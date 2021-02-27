function qs = DampedLeastSquares(current_config, trajectory, l, myo)

    sp = 350;
    
    jn = 7; % number of joints in the Serial manipulator  % Change Here
    i = 1;
    
    current = FK(l, current_config, 0, "0 0 0");
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

    
    nPoints = size(trajectory, 2);
    for k=1:nPoints
        final = trajectory(:,k);
        while (sum(abs(final - current)) > 50)
            
            if ~ishandle(Robot), return, end
            delete([links_plot,joints_plot,end_effector_plot])
            delete(axes_plot)
            if k==1
                delete(path_plot)
            end

            Task_step = (final - current);
            Task_step = sp * Task_step/norm(Task_step);
%             new = current + Task_step;

        %%% Computing J_DLS
            J = Jacobian(l, q);           % Change Here

            [U,S,V] = svd(J) ;
            eig_val = diag(S); 
            S = zeros(size(J,2),size(J,1)); 
            for e = 1:length(eig_val)
                v(e) = eig_val(e)/((eig_val(e))^2 + myo^2);
            end 
            S(1:size(J,1),1:size(J,1)) = diag(v) ;
        %%% --------------

            J_DLS = V * S * U';
            Joint_step = J_DLS * Task_step;
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
%             daspect([1 1 1])
            drawnow

        end
    end

end
