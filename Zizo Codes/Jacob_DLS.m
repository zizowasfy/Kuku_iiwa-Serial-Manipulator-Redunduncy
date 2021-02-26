function J_DLS = Jacob_DLS(J)
    myo = 1.5 ;
    [U,S,V] = svd(J) ;
    eig_val = diag(S); %[S(1,1) S(2,2) S(3,3)];
    S = zeros(size(J,2),size(J,1)); %zeros(4,3)
    for i = 1:length(eig_val)
        v(i) = eig_val(i)/((eig_val(i))^2 + myo^2);
    end 
    S(1:size(J,1),1:size(J,1)) = diag(v) ; % ([eig_val(1)/((eig_val(1))^2 + myo^2) , eig_val(2)/((eig_val(2))^2 + myo^2), eig_val(3)/((eig_val(3))^2 + myo^2) ]) ;
    
    J_DLS = V * S * U';
end