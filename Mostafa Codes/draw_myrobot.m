function draw_myrobot(T)
% FK=simplify(Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2)*Rz(q3)*Tz(L3)*Rx(q4)*Tz(L4)*Rz(q5)*Tz(L5)*Rx(q6)*Tz(L6)*Rz(q7)*Tz(L7))


% 
% hold on
% view([-179.88 -90.00])
% grid on
% xlim([-2.21 2.79])
% ylim([-2.08 2.92])
% zlim([-1.00 0.50])

title('Kuka iiwa Robot')

plot3(0,0,0,'ro','MarkerSize',5,'LineWidth', 5);
hold on

for i = 1:((length(T) )-1)
    T1 = cell2mat(T(i));
    T2 = cell2mat(T(i+1));
    if i == 1 
        plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-b','LineWidth', 3);
        plot3(T1(1,4),T1(2,4),T1(3,4),'ro','MarkerSize',3,'LineWidth', 3);
        plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-b','LineWidth', 3);
        plot3(T2(1,4),T2(2,4),T2(3,4),'ro','MarkerSize',3,'LineWidth', 3);  
    else  
        plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-b','LineWidth', 3);
        plot3(T2(1,4),T2(2,4),T2(3,4),'ro','MarkerSize',3,'LineWidth', 3);
    end
    hold on
end
Tor = cell2mat(T(length(T)));
T5x=Tor *Tx( 2);
quiver3(Tor(1,4),Tor(2,4),Tor(3,4),T5x(1,4)-Tor(1,4),T5x(2,4)-Tor(2,4),T5x(3,4)-Tor(3,4),0,'-p');
T5y=Tor *Ty( 2);
quiver3(Tor(1,4),Tor(2,4),Tor(3,4),T5y(1,4)-Tor(1,4),T5y(2,4)-Tor(2,4),T5y(3,4)-Tor(3,4),0,'-c');
T5z=Tor *Tz( 2);
quiver3(Tor(1,4),Tor(2,4),Tor(3,4),T5z(1,4)-Tor(1,4),T5z(2,4)-Tor(2,4),T5z(3,4)-Tor(3,4),0,'-g');
hold on


