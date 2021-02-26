function dq0 = calc_dQ0(l, q_current, objective, jointsRange)

dq = 0.01; % small value added to compute the derivatives numerically
k0 = 0.00000001;

% this is related to range objective function
qm = jointsRange(:,1);
qM = jointsRange(:,2);
q_bar = (qm + qM)/2;

J = Jacobian(l, q_current);
N = length(q_current);

% objective function:
if objective == "man"
    H_man = sqrt( det(J*J') );
elseif objective == "range"
    accumelator = 0;
    for j=1:N
        accumelator = accumelator + ( (q_current(j) - q_bar(j))/(qM(j) - qm(j)) )^2;
    end
    H_man = accumelator/(2*N);
end

for i=1:N
    q_modified = q_current;
    q_modified(i) = q_modified(i) + dq;

    J_modified = Jacobian(l, q_modified);
    
    % objective function:
    if objective == "man"
        H_man_modified = sqrt( det(J_modified*J_modified') );
        gradient = (H_man_modified - H_man)/dq;
    elseif objective == "range"
        accumelator = 0;
        for j=1:N
            accumelator = accumelator + ( (q_modified(j) - q_bar(j))/(qM(j) - qm(j)) )^2;
        end
        H_man_modified = accumelator/(2*N);
        gradient = -(H_man_modified - H_man)/dq;
    end

    dq0(i) = k0*gradient;
end

dq0 = dq0';