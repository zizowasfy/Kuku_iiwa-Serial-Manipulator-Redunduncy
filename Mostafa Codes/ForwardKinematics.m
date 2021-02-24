function [FK,T] = ForwardKinematics(q,L)

q1 = q(:,1);q2 = q(:,2);q3 = q(:,3);q4 = q(:,4);q5 = q(:,5);q6 = q(:,6);q7 = q(:,7);
L1 = L(:,1);L2 = L(:,2);L3 = L(:,3);L4 = L(:,4);L5 = L(:,5);L6 = L(:,6);L7 = L(:,7);
 
T1 = mat2cell(Rz(q1)*Tz(L1),4);
T2 = mat2cell(Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2),4);
T3 = mat2cell(Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2)*Rz(q3)*Tz(L3),4);
T4 = mat2cell(Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2)*Rz(q3)*Tz(L3)*Rx(q4)*Tz(L4),4);
T5 = mat2cell(Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2)*Rz(q3)*Tz(L3)*Rx(q4)*Tz(L4)*Rz(q5)*Tz(L5),4);
T6 = mat2cell(Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2)*Rz(q3)*Tz(L3)*Rx(q4)*Tz(L4)*Rz(q5)*Tz(L5)*Rx(q6)*Tz(L6),4);
T7 = Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2)*Rz(q3)*Tz(L3)*Rx(q4)*Tz(L4)*Rz(q5)*Tz(L5)*Rx(q6)*Tz(L6)*Rz(q7)*Tz(L7);

Tf=T7(1:3,4);
T7 = mat2cell(Rz(q1)*Tz(L1)*Rx(q2)*Tz(L2)*Rz(q3)*Tz(L3)*Rx(q4)*Tz(L4)*Rz(q5)*Tz(L5)*Rx(q6)*Tz(L6)*Rz(q7)*Tz(L7),4);

px=Tf(1);
py=Tf(2);
pz=Tf(3);
 
FK=[px;py;pz]';

T=[T1;T2;T3;T4;T5;T6;T7];
end
% px = cos(q1)* (L2*cos(q2)+ L3*cos(q2+q3));
% py = sin(q1)* (L2*cos(q2)+ L3*cos(q2+q3));
% pz = L1+ L2*sin(q1)+ L3*sin(q2+q3);