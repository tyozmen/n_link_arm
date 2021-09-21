function [J] = nlink_Jacobian(Q,L)
n = length(L);
q = Q(1:n,1);
J = zeros(3,n);

for i = 1:n
    J(1,i) = L(i)*cos(q(i));
    J(2,i) = -L(i)*sin(q(i));
end
J(3,n) = 1;

end

