clear all; clc; close all;
% In this Code: direction signs are inverted since the robot reference is
% not as the convention.
% Clockwise -> (+ve) and Anti-Clockwise -> (-ve)
%% Robot 
L = 1 ;
T_base = eye(4) ;
q = [deg2rad(0), deg2rad(90), deg2rad(90), deg2rad(-90)];
% Forward Kinematics 
FK = localFK(q,L);
% Jacobian
Jacobian = Jacob(T_base, FK, q, L);
W = eye(4) ;
% RObot Visualization
% figure
% robot(L,q)
%

%% PsuedoInverse
% q = [deg2rad(90), deg2rad(0), deg2rad(0), deg2rad(90)];        % Desired Position in Joint Space
% FK = localFK(q,L);
% final_pos = FK(1:2,4); % [x,y]  position in x,y plane
% final_ori = atan2(FK(2,1), FK(1,1)); % [phi] orientation around z
% final = [final_pos', final_ori]
% 
% q = [deg2rad(5), deg2rad(30), deg2rad(0), deg2rad(0)]';          % Initial Position in Joint Space
% FK = localFK(q,L);
% initial_pos = FK(1:2,4);
% initial_ori = atan2(FK(2,1), FK(1,1));
% initial = [initial_pos', initial_ori]
% Jacobian = Jacob(T_base, FK, q, L)
% 
% current = initial;
% % step = (final - initial)/10 ;
% i = 1;
% while (sum(abs(final - current)) > 0.001)
%     
%     Task_step = (final - current)/10; %0.01*ones(1,3);
%     new = current + Task_step;
%     [J,J_hash] = Jacob_Based(Jacobian, "pinv", W);
%     Joint_step = J_hash * Task_step';
%     qs(1:4,i) = q + Joint_step;
% 
% % visualizing the Robot's Path
%     clf
%     robot(L,qs(:,i));
%     pause(0.01)
%     
% % updating the variables  
%     q = qs(1:4,i);
%     FK = localFK(q,L);
%     Jacobian = Jacob(T_base, FK, q, L);
%     current = [FK(1:2,4)', atan2(FK(2,1), FK(1,1))];
%     i = i + 1;
%     
% end
% 
%% Task Priority Augmentation  - 
% q = [deg2rad(170), deg2rad(0), deg2rad(-30), deg2rad(0)];        % Desired Position in Joint Space
% FK = localFK(q,L);
% final_pos = FK(1:2,4); % [x,y]  position in x,y plane
% final_ori = atan2(FK(2,1), FK(1,1)); % [phi] orientation around z
% final = [final_pos', final_ori]

q = [deg2rad(0), deg2rad(90), deg2rad(90), deg2rad(-90)]';          % Initial Position in Joint Space
FK = localFK(q,L);
initial_pos = FK(1:2,4);
initial_ori = atan2(FK(2,1), FK(1,1));
initial = [initial_pos', initial_ori]
Jacobian = Jacob(T_base, FK, q, L);

current = initial;

% square path
[xs,ys,dim,n] = deal(current(1), current(2), 2, 100);
[xsq,ysq] = generateSquare(xs,ys,dim,n);

final = [xsq(n) ysq(n) current(3)];
i = 1;
% r1 = [xsq(2)-xsq(1) ysq(2)-ysq(1)];
% r1 = r1/0.005*norm(r1);
while (sum(abs(final - current)) > 0.001)

    r1 = (final(1:2) - current(1:2))/10;  
    [J1,J_hash] = Jacob_Based(Jacobian, "pinv", W);  %J_DLS = Jacob_DLS(J1);
    J2 = Jacobian(6,:) 
    r2 = 0;             % sum of joints = 0 ( Task 2 )
    Joint_step = J_hash * r1' + pinv((J2*(eye(4) - J_hash*J1))) * (r2' - J2*J_hash*r1');
    qs(1:4,i) = q + Joint_step;

% visualizing the Robot's Path
    clf
    robot(L,qs(:,i));
    pause(0.01)
    
% updating the variables  
    q = qs(1:4,i);
    FK = localFK(q,L);
    Jacobian = Jacob(T_base, FK, q, L);
    current = [FK(1:2,4)', atan2(FK(2,1), FK(1,1))]
    i = i + 1;
    
end
%% visualizing the Robot's Path
% close all;
% figure
% for i = 1:10:length(qs)
%     clf
%     robot(L,qs(:,i));
%     xlim([-4.74 4.26])
%     ylim([-3.83 5.17])
%     zlim([-3.50 5.50])
%     view([-178.85 90.00])
%     pause(0.01)
% end
%% Weighted 
% q = [deg2rad(90), deg2rad(-90), deg2rad(90), deg2rad(90)];        % Desired Position in Joint Space
% FK = localFK(q,L);
% final_pos = FK(1:2,4); % [x,y]  position in x,y plane
% final_ori = atan2(FK(2,1), FK(1,1)); % [phi] orientation around z
% final = [final_pos', final_ori]
% 
% q = [deg2rad(10), deg2rad(0), deg2rad(0), deg2rad(-90)]';       % Initial Position in Joint Space
% FK = localFK(q,L);
% initial_pos = FK(1:2,4);
% initial_ori = atan2(FK(2,1), FK(1,1));
% initial = [initial_pos', initial_ori]
% 
% Jacobian = Jacob(T_base, FK, q, L)
% 
% current = initial;
% 
% W = diag([2, 2, 1, 2]);
% i = 1;
% while (sum(abs(final - current)) > 0.001)
%     
%     Task_step = (final - current)/10;
%     new = current + Task_step;
%     
%     J_hash_W = Jacob_Based(Jacobian, "weighted pinv", W);
%     Joint_step = J_hash_W * Task_step';
%     qs(1:4,i) = q + Joint_step;
% 
% % visualizing the Robot's Path
%     clf
%     robot(L,qs(:,i));
%     pause(0.01)
%     
% % updating the variables  
%     q = qs(1:4,i);
%     FK = localFK(q,L);
%     Jacobian = Jacob(T_base, FK, q, L);
%     current = [FK(1:2,4)', atan2(FK(2,1), FK(1,1))];
%     i = i + 1;
%     
% end
%% Damped Least Square (DLS)
% J = [Jacobian(1:2,:) ; Jacobian(6,:)] ; 
% myo = 1.5 ;
%  
% %%%%%%%%%%%%%%
% 
% q = [deg2rad(90), deg2rad(0), deg2rad(0), deg2rad(45)];        % Desired Position in Joint Space
% FK = localFK(q,L);
% final_pos = FK(1:2,4); % [x,y]  position in x,y plane
% final_ori = atan2(FK(2,1), FK(1,1)); % [phi] orientation around z
% final = [final_pos', final_ori]
% 
% q = [deg2rad(5), deg2rad(0), deg2rad(0), deg2rad(0)]';          % Initial Position in Joint Space
% FK = localFK(q,L);
% initial_pos = FK(1:2,4);
% initial_ori = atan2(FK(2,1), FK(1,1));
% initial = [initial_pos', initial_ori]
% Jacobian = Jacob(T_base, FK, q, L)
% 
% current = initial;
% % step = (final - initial)/10 ;
% i = 1;
% while (sum(abs(final - current)) > 0.001)
%     
%     Task_step = (final - current)/10;
%     new = current + Task_step;
% 
%     J = [Jacobian(1:2,:) ; Jacobian(6,:)] ; 
%     [U,S,V] = svd(J) ;
%     eig_val = [S(1,1) S(2,2) S(3,3)];
%     S = zeros(4,3);
%     S(1:3,1:3) = diag([ eig_val(1)/((eig_val(1))^2 + myo^2) , eig_val(2)/((eig_val(2))^2 + myo^2) , eig_val(3)/((eig_val(3))^2 + myo^2) ]) ;
%     
%     J_DLS = V * S * U';
%     Joint_step = J_DLS * Task_step';
%     qs(1:4,i) = q + Joint_step;
% 
% % visualizing the Robot's Path
%     clf
%     robot(L,qs(:,i));
%     pause(0.01)
%     
% % updating the variables  
%     q = qs(1:4,i);
%     FK = localFK(q,L);
%     Jacobian = Jacob(T_base, FK, q, L);
%     current = [FK(1:2,4)', atan2(FK(2,1), FK(1,1))];
%     i = i + 1;
%     
% end
%%

