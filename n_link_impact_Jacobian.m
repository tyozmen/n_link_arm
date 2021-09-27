function [J] = n_link_impact_Jacobian(Q,L, x_imp, y_imp)

%%%%%%%%%%%%%%%%%%%%%%% JUST USE nlink Jacobian with the length output of n_link_impact_point %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculates the Jacobian considering the point of impact as the end
% effector

n = length(L);
q = Q(1:n,1);
J = zeros(3,n);
ee = n_link_fwdKin(Q(1:n),L);

xn = ee(1) - L(n)*sin(Q(n));    % x coordinate of nth link
yn = ee(2) - L(n)*cos(Q(n));    % x coordinate of nth link

l_imp = sqrt((xn - x_imp)^2+(yn-y_imp)^2);

for i = 1:n-1
    J(1,i) = L(i)*cos(q(i));
    J(2,i) = -L(i)*sin(q(i));
end
J(1,n) = l_imp*cos(q(n));
J(2,n) = -l_imp*sin(q(n));

J(3,n) = 1;

end