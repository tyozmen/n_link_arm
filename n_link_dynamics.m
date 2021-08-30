function [dQ] = n_link_dynamics(Q,M_fnc,C_fnc,Tg_fnc,B,g,q_vars,full_vrs)
n = length(Q)/2;

u = zeros(n,1);
dq_act(1:n,1) = Q(n+1:end);
q_act(1:n,1) = Q(1:n);
% for k=1:n
%     eval(sprintf('q%d = q_act(k);', k));
%     eval(sprintf('dq%d = dq_act(k);', k));
% end
% M = double(subs(M)); C = double(subs(C)); Tg = double(subs(Tg)); 
eval(sprintf(['Tg = Tg_fnc(' q_vars ');']));
eval(sprintf(['M = M_fnc(' q_vars ');']));
eval(sprintf(['C = C_fnc(' full_vrs ');']));

d2q = double(-M\C*dq_act-M\Tg*g+M\B*u);
dQ = [dq_act; d2q];
end

