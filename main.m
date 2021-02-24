clear all; clc; close all;
% In this Code: direction signs are inverted since the robot reference is
% not as the convention.
% Clockwise -> (+ve) and Anti-Clockwise -> (-ve)
%% Robot 
L = 1 ;
T_base = eye(4) ;
q = [deg2rad(5), deg2rad(0), deg2rad(0), deg2rad(45)];
% Forward Kinematics 
FK = localFK(q,L);
% Jacobian
Jacobian = Jacob(T_base, FK, q, L)
% RObot Visualization
% figure
% robot(L,q)
%%
% Jacobian Based Methods

% %% PsuedoInverse
% q = [deg2rad(90), deg2rad(0), deg2rad(0), deg2rad(45)];        % Desired Position in Joint Space
% FK = localFK(q,L);
% final_pos = FK(1:2,4); % [x,y]  position in x,y plane
% final_ori = atan2(FK(2,1), FK(1,1)); % [phi] orientation around z
% final = [final_pos', final_ori]
% 
% q = [deg2rad(0), deg2rad(0), deg2rad(0), deg2rad(0)]';       % Initial Position in Joint Space
% FK = localFK(q,L);
% initial_pos = FK(1:2,4);
% initial_ori = atan2(FK(2,1), FK(1,1));
% initial = [initial_pos', initial_ori]
% 
% current = initial;
% % step = (final - initial)/10 ;
% i = 1;
% while (sum(abs(final - current)) > 0.001)
%     
%     Task_step = (final - current)/10;
%     new = current + Task_step;
%     J_hash = Jacob_Based(Jacobian, "pinv", W);
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

%%
% % visualizing the Robot's Path
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
q = [deg2rad(90), deg2rad(-90), deg2rad(90), deg2rad(90)];        % Desired Position in Joint Space
FK = localFK(q,L);
final_pos = FK(1:2,4); % [x,y]  position in x,y plane
final_ori = atan2(FK(2,1), FK(1,1)); % [phi] orientation around z
final = [final_pos', final_ori]

q = [deg2rad(10), deg2rad(0), deg2rad(0), deg2rad(-90)]';       % Initial Position in Joint Space
FK = localFK(q,L);
initial_pos = FK(1:2,4);
initial_ori = atan2(FK(2,1), FK(1,1));
initial = [initial_pos', initial_ori]

Jacobian = Jacob(T_base, FK, q, L)

current = initial;

W = diag([2, 2, 1, 2]);
i = 1;
while (sum(abs(final - current)) > 0.001)
    
    Task_step = (final - current)/10;
    new = current + Task_step;
    
    J_hash_W = Jacob_Based(Jacobian, "weighted pinv", W);
    Joint_step = J_hash_W * Task_step';
    qs(1:4,i) = q + Joint_step;

% visualizing the Robot's Path
    clf
    robot(L,qs(:,i));
    pause(0.01)
    
% updating the variables  
    q = qs(1:4,i);
    FK = localFK(q,L);
    Jacobian = Jacob(T_base, FK, q, L);
    current = [FK(1:2,4)', atan2(FK(2,1), FK(1,1))];
    i = i + 1;
    
end

