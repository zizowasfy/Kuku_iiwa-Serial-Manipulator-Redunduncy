function robot(L,q)

% T_H = RotR("z",theta1) * Trans("z",L1) * RotR("y",theta2) * Trans("x",L2) * RotR("y",theta3) * Trans("x",L3) 
% v  = [theta1 theta2 d3]

% figure;




T1 = RotR("z",q(1)) * Transl("x",L);
plot3(0,0,0,'ro','MarkerSize',5,'LineWidth', 5 , 'DisplayName' , 'Joint 0');

hold on
axis([-1 3 -1 3 -1 3])    % Question 3
% view(20,12)           % Question 3
view([-23.25 35.19])
xlabel("x-axis")
ylabel("y-axis")
zlabel("z-axis")
grid on

plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-b','LineWidth', 3 , 'DisplayName' , 'Link 1');
plot3(T1(1,4),T1(2,4),T1(3,4),'ro','MarkerSize',5,'LineWidth', 5 , 'DisplayName' , 'Joint 1');


T2 = RotR("z",q(1)) * Transl("x",L) * RotR("z",q(2)) * Transl("x",L) ;
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-g','LineWidth', 4 , 'DisplayName' , 'Link 2');
plot3(T2(1,4),T2(2,4),T2(3,4),'ro','MarkerSize',5,'LineWidth', 5 , 'DisplayName' , 'Joint 2');


T3 = RotR("z",q(1)) * Transl("x",L) * RotR("z",q(2)) * Transl("x",L) * RotR("z",q(3)) * Transl("x",L) ;
plot3([T2(1,4) T3(1,4)],[T2(2,4) T3(2,4)],[T2(3,4) T3(3,4)],'-m','LineWidth', 5 , 'DisplayName' , 'Link 3');
plot3(T3(1,4),T3(2,4),T3(3,4),'ro','Markersize',5,'LineWidth',5 , 'DisplayName' , 'End-Effector');

T4 = RotR("z",q(1)) * Transl("x",L) * RotR("z",q(2)) * Transl("x",L) * RotR("z",q(3)) * Transl("x",L) * RotR("z",q(4)) * Transl("x",L) ;
plot3([T3(1,4) T4(1,4)],[T3(2,4) T4(2,4)],[T3(3,4) T4(3,4)],'-m','LineWidth', 5 , 'DisplayName' , 'Link 3');
plot3(T4(1,4),T4(2,4),T4(3,4),'ro','Markersize',5,'LineWidth',5 , 'DisplayName' , 'End-Effector');
% plot3([T4(1,4) T4(1,4)+1],[T4(2,4) T4(2,4)+1],[T4(3,4) T4(3,4)+1], 'r-', 'Markersize', 1,'LineWidth',1 , 'DisplayName' , 'End-Effector')
legend;

% xlim([-4.74 4.26])
% ylim([-3.83 5.17])
% zlim([-3.50 5.50])
% view([-178.85 90.00])  
xlim([-5 5])
ylim([-5 5])
zlim([-3.50 5.50])
view([180.69 -90.00])

end


