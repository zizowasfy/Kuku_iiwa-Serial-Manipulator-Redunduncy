function DampedLeastSquares(final, current, Jacobian, q, L, myo)

    jn = 4; % number of joints in the Serial manipulator  % Change Here
    i = 1;
    while (sum(abs(final - current)) > 0.001)

        Task_step = (final - current)/10;
        new = current + Task_step;

    %%% Computing J_DLS
        J = [Jacobian(1:2,:) ; Jacobian(6,:)] ;           % Change Here
        [U,S,V] = svd(J) ;
        eig_val = diag(S); 
        S = zeros(size(J,2),size(J,1)); 
        for i = 1:length(eig_val)
            v(i) = eig_val(i)/((eig_val(i))^2 + myo^2);
        end 
        S(1:size(J,1),1:size(J,1)) = diag(v) ;
    %%% --------------

        J_DLS = V * S * U';
        Joint_step = J_DLS * Task_step';
        qs(1:jn,i) = q + Joint_step;                      

    % visualizing the Robot's Path
        clf
        robot(L,qs(:,i));
        pause(0.01)

    % updating the variables    
        q = qs(1:jn,i);                                  
        FK = localFK(q,L);
        Jacobian = Jacob(eye(4), FK, q, L);
        current = [FK(1:2,4)', atan2(FK(2,1), FK(1,1))];
        i = i + 1;

    end

end