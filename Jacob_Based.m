function [J, J_hash] = Jacob_Based(Jacobian, type, W)
J = [Jacobian(1:2,:); Jacobian(6,:)] ;     % x,y,phi rows
J;
if type == "pinv" 
    J_hash = J' * inv(J*J') ;
    
elseif type == "weighted pinv" 
    J_hash = inv(W)* J' * inv(J*inv(W)*J') ; 
end 

end