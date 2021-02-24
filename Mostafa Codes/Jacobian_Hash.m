function J_hash = Jacobian_Hash(q,L,W,type)
J = JacobianNumerical(L,q)
% J = [Jq(1:2,:); Jq(6,:)]

if type == "hashJ"
    J_hash = J' * inv(J*J');
elseif type == "PseudoJ"
    J_hash = J' * pinv(J*J');
elseif type == "weighted_pseudoJ"
    J_hash = W'*J' * pinv(J*W'*J');
else
    display("error")
end

end
