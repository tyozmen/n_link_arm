function [dQ] = n_link_dynamics_FeedbackLinearization(Q,M_fnc,C_fnc,Tg_fnc,B,g)
n = length(Q)/2;

dq_act(1:n,1) = Q(n+1:end);
q_act(1:n,1) = Q(1:n);
Kp = 25; Kd = 4;
% for k=1:n
%     eval(sprintf('q%d = q_act(k);', k));
%     eval(sprintf('dq%d = dq_act(k);', k));
% end
% M = double(subs(M)); C = double(subs(C)); Tg = double(subs(Tg)); 
Tg = Tg_fnc(q_act);
M = M_fnc(q_act);
C = C_fnc(Q);

u_fl = double(C*dq_act + Tg*g);
u = u_fl+Kp.*rad2deg(-q_act) + Kd*rad2deg(-dq_act);
d2q = double(-M\C*dq_act-M\Tg*g+M\B*u);
dQ = [dq_act; d2q];
end

