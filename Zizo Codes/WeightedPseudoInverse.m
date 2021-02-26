function WeightedPseudoInverse(final, current, Jacobian, q, L, W)

    jn = 4; % number of joints in the Serial manipulator  % Change Here
    i = 1;
    while (sum(abs(final - current)) > 0.001)

        Task_step = (final - current)/10;
        new = current + Task_step;

    %%% Computing J_hash_Weighted
        J = [Jacobian(1:2,:) ; Jacobian(6,:)] ;           % Change Here
        J_hash_W = inv(W)* J' * inv(J*inv(W)*J') ; 

        Joint_step = J_hash_W * Task_step';
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