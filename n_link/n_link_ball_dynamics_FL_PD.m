function [dQ] = n_link_ball_dynamics_FL_PD(Q,M_fnc,C_fnc,Tg_fnc,B,g)

% n_link arm an ball dynamics

% states: q1-qn,xb,yb,dq1-dqn-dxb,dyb


n = length(B);
q_des = [pi/4; -pi/4; pi/2];

% Arm states
dq_act(1:n,1) = Q(n+3:end-2);
q_act(1:n,1) = Q(1:n);

% ball states
x_b = Q(n+1);
y_b = Q(n+2);
dx_b = Q(end-1);
dy_b = Q(end);

Kp = 15; Kd = 1;

Tg = Tg_fnc(q_act);
M = M_fnc(q_act);
C = C_fnc(Q);

u_fl = double(C*dq_act + Tg*g);
u = u_fl+Kp.*rad2deg(q_des-q_act) + Kd*rad2deg(-dq_act); % FL and PD control

d2q = double(-M\C*dq_act-M\Tg*g+M\B*u);
dQ = [dq_act; dx_b; dy_b; d2q; 0; -g];

end