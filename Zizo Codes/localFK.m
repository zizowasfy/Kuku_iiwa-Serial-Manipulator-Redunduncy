function T_EE = localFK(q, L)

T_J1 = RotR("z",q(1)) ; % Transformation from robot local origin to joint 1
T_J2 = RotR("z",q(1)) * Transl("x",L) * RotR("z",q(2)) ;  % Transformation from robot local origin to joint 2
T_J3 = RotR("z",q(1)) * Transl("x",L) * RotR("z",q(2)) * Transl("x",L) * RotR("z",q(3)) ;
T_J4 = RotR("z",q(1)) * Transl("x",L) * RotR("z",q(2)) * Transl("x",L) * RotR("z",q(3)) * Transl("x",L) * RotR("z",q(4));
T_EE = RotR("z",q(1)) * Transl("x",L) * RotR("z",q(2)) * Transl("x",L) * RotR("z",q(3)) * Transl("x",L) * RotR("z",q(4)) * Transl("x",L) ;

T_local = T_EE;

end