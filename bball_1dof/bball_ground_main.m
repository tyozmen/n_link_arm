clear all
x0 = [0; 4; 0; 0; 9];
m = 0.25;
r = 0.1;
e = .8;
g = 9.81;
opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30,'Events',@(t,X)contact_ground(t,X,r)); 
[t1,y1]= ode45(@(t1,Q)bball_dynamics(Q,g),[0 4],x0,opts);
% bball_ground_animate(t1,y1,r)
t=[];
y=[];
for i = 1:10
    t = [t;t1(2:end)];
    y = [y;y1(2:end,:)];
    v_xi = y1(end,3); 
    v_yi = y1(end,4);
    wi = y1(end,5);
    wf = (3*m*v_xi+2*m*r*wi)/(r*2*m*3);
    v_xf = r*wf;
    v_yf = -e*v_yi;
    q = [y(end,1); y(end,2); v_xf; v_yf; wf];
    [t1,y1]= ode45(@(t1,Q)bball_dynamics(Q,g),[t(end) t(end)+4],q,opts);
end
bball_ground_animate(t,y,r)