function Jq = Jacobian(L,q)

% Calculating Jacobian by numerical Method
% syms q1 q2 q3 q4 L1 L2 L3 L4 real
% L1 = 1; L2 = 1;L3 = 1;L4 = 1;
% q1 = deg2rad(90);q2 = deg2rad(0); q3 = deg2rad(0); q4 = deg2rad(0);
% 

% H=simplify(Rz(q1)*Tx(L1)*Rz(q2)*Tx(L2)*Rz(q3)*Tx(L3)*Rz(q4)*Tx(L4))
% 
% R = simplify(H(1:3,1:3))

%Same could be done for other Jacobian columns
q = deg2rad(q);

% forward kinematics
H = Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7));
% H=simplify(H);
% extract rotation matrix
R = H(1:3,1:3);

% diff by q1
Td=Rzd(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
    [R^-1 zeros(3,1);0 0 0 1];
% extract 6 components from 4x4 Td matrix to Jacobian 1st column
J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% diff by q2
Td=Rz(q(1))*Tz(L(1))*Rxd(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
    [R^-1 zeros(3,1);0 0 0 1];
% extract 6 components from 4x4 Td matrix to Jacobian 2nd column
J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% diff by q3
Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rzd(q(3))*Tz(L(3))*Rx(q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
    [R^-1 zeros(3,1);0 0 0 1];
% extract 6 components from 4x4 Td matrix to Jacobian 3rd column
J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% diff by q4
Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rxd(q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7)) *...
    [R^-1 zeros(3,1);0 0 0 1];
% extract 6 components from 4x4 Td matrix to Jacobian 3rd column
J4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% diff by q5
Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(q(4))*Tz(L(4))*Rzd(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7)) *...
    [R^-1 zeros(3,1);0 0 0 1];
% extract 6 components from 4x4 Td matrix to Jacobian 3rd column
J5 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% diff by q6
Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rxd(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7)) *...
    [R^-1 zeros(3,1);0 0 0 1];
% extract 6 components from 4x4 Td matrix to Jacobian 3rd column
J6 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

% diff by q7
Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rzd(q(7))*Tz(L(7)) *...
    [R^-1 zeros(3,1);0 0 0 1];
% extract 6 components from 4x4 Td matrix to Jacobian 3rd column
J7 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;


Jq = [J1,J2,J3,J4,J5,J6,J7];
% Jq = [simplify(J1), simplify(J2), simplify(J3),simplify(J4)]

end
