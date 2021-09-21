function [dQ] = bball_dynamics(Q,g)
x = Q(1);
v = Q(2);
w= Q(3);
dQ = [Q(3); Q(4); 0; -g; 0];
end