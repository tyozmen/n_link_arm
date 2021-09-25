clear all
%%%% TESTING THE 1 DOF SYSTEM 
%%%% run it to see test animation
% states: [y_s; y_b; dy_s; dy_b]

x0 = [0; 3; 0; 0];
m_b = 0.25;
m_s = 1;
r = 0.1;        % radius of the ball
d = 0.05;   % half thickness of the link
e = .8;
g = 9.81;


opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30,'Events',@(t,X)contact_link_1_dof(t,X,r,d)); 
[t1,y1]= ode45(@(t1,Q)bball_1_dof_dynamics(Q,g),[0 4],x0,opts);
% plot(t1,y1(:,1),t1,y1(:,3));

% bball_1_dof_animate(t1,y1,r)

t=t1(1);
y=y1(1,:);
for i = 1:10
    t = [t;t1(2:end)];
    y = [y;y1(2:end,:)];
    dy_si = y1(end,3); 
    dy_bi = y1(end,4);
    
    dy_bf = -e*(dy_bi-dy_si)+dy_si; %%%%%%%%%%%%%55
    dy_sf = (1+e)*m_b*(dy_bi-dy_si)/m_s + dy_si;
   
    q = [y(end,1); y(end,2); dy_sf; dy_bf];
    [t1,y1]= ode45(@(t1,Q)bball_1_dof_dynamics(Q,g),[t(end) t(end)+4],q,opts);
end
bball_1_dof_animate(t,y,r)