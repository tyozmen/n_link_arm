function [dQ] = bball_1_dof_dynamics(Q,g)
y_s = Q(1);
y_b = Q(2);
dy_s = Q(3);
dy_b = Q(4);

m_s = 1;    % mass of the link
m_b = .25;  % mass of the ball

kp = 65;
kd = 4.5;


%PD control to keep the link at y = 2 (cancels effect of gravity as well)
u = m_s*g+kp*(2-y_s)+kd*(0 - dy_s);      

dQ = [dy_s; dy_b; u/m_s-g; -g];
end

