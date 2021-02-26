function TaskAugmentation(current, Jacobian, q, L, xsq, ysq, n)

    jn = 4;     % number of joints in the Serial manipulator            % Change Here

    for p = 1:4*n

        final = [xsq(p) ysq(p)];
        i = 1;
        while (sum(abs(final - current(1:2))) > 0.001)

            r1 = (final(1:2) - current(1:2))/1;                         % Change Here
            J1 = Jacobian(1:2,:) ;                                      % Change Here  
            J_hash = J1' * inv(J1*J1') ;  
            J2 = [1 1 1 1]%Jacobian(6,:);                                         % Change Here  
            r2 = 0;              % sum of joints = 0 ( Task 2 )         % Change Here
            Joint_step = J_hash * r1' + pinv((J2*(eye(4) - J_hash*J1))) * (r2' - J2*J_hash*r1');
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

end