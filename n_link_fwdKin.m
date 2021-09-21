function [ee] = n_link_fwdKin(Q,L)
%where Q are the states and L is the vector of arm lengths

n = length(L);
q = Q(1:n,1);
x_ee  = L'*sin(q);
y_ee = L'*cos(q);
th_ee = q(end);
ee = [x_ee; y_ee; th_ee];
end

