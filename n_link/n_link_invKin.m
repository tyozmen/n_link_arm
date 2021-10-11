function [Q] = n_link_invKin(x_ee,y_ee,th_ee,L)
ee = [x_ee; y_ee; th_ee];
n = length(L);
q_old = zeros(2*n,1);
q_new = zeros(2*n,1);
e = sum((ee-n_link_fwdKin(q_old,L)).^2);
% search for joint angles that minimizes the error between the fwdKin and 
% the desired ee coordinates
while e > 1e-8
% for j = 1:100 %iterate for 100 times
    for i = 1:n
        if i == 1
           fun = @(q_tmp)sum((ee-n_link_fwdKin([q_tmp; q_old(2:end,1)],L)).^2);
           q = fminbnd(fun,-pi,pi);
           q_new(i,1) = q;
        else
           fun = @(q_tmp)sum((ee-n_link_fwdKin([q_old(1:i-1,1);q_tmp;q_old(i+1:end,1)],L)).^2);
           q = fminbnd(fun,q_old(i-1)-pi,q_old(i-1)+pi);
           q_new(i,1) = q;
        end
        q_old = q_new;
        e = sum((ee-n_link_fwdKin(q_old,L)).^2);
    end
end
Q = q_new;
end

