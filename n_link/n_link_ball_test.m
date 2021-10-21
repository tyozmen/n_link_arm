clear all
n = 3;
[M_sym,C_sym,Tg_sym,B,param] = manipEqns(n);
L = param.L;
[M,C,Tg] = sym2anonFnc(M_sym,C_sym,Tg_sym); %get M,C,Tg as anonymous functions
g = 9.81;

r = 0.1;    % radius of the ball
d = 0.05;   % half thickness of the link
e = .8;
m_b = 0.15;
%% Run ODE CAUTION!!!!!!!!! Change initial conditions for ODE if 'n' is changed 


% Feedback Linearization to cancel the effects of gravity and friction. Arm
% should be able stay at initial conditions.
x0 = [pi/4 -pi/4 pi/2 0.5 4 0 0 0 0 0];
opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30,'Events',@(t,X)contact_n_link_ball_v2(t,X,L,r,d)); 
% opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30);
[t1,y1]= ode45(@(t1,Q)n_link_ball_dynamics_FL_PD(Q,M,C,Tg,B,g),[0 4],x0,opts);
% n_link_ball_animate(t1,y1,L,r,d)



t=t1(1);
y=y1(1,:);
for i = 1:25
    if t1(end) == t(end)+4
        break
    end
    t = [t;t1(2:end)];
    y = [y;y1(2:end,:)];

    [x_imp,y_imp,l_imp,th] = n_link_impact_point(y1(end,:)',L,r,d);    % get the impact point and the distance it happens from the nth link, th is the surface angle wrt x-axis
    J_imp = nlink_Jacobian(y1(end,:)',[L(1:n-1,1); l_imp]);      % get the Jacobian assuming the impact point is the end effector
    
    dz_imp_i = J_imp*y1(end,n+3:end-2)'; % impact point velocities right before impact
    
    dx_si = dz_imp_i(1);  % x velocity of the surface right before impact
    dy_si = dz_imp_i(2);  % y velocity of the surface right before impact
    
    dx_bi = y1(end,end-1);  % x velocity of the ball right before impact
    dy_bi = y1(end,end);    % y velocity of the ball right before impact
    
    % need to rotate the coordinate system so our basis vectors are
    % perpendicular and parallel to the surface
    R = [cos(th) sin(th); -sin(th) cos(th)];
    Vr_bi = R*[dx_bi; dy_bi]; 
    Vr_si = R*[dx_si; dy_si];
    
    Vr_bf = zeros(2,1); % post impact ball velocities in the rotated coordinate system
    
%     Vr_bf(2) = -e*(Vr_bi(2)-Vr_si(2)) + Vr_si(2); % post-impact y velocity of ball
%     N = m_b*(Vr_bf(2) - Vr_bi(2)); % impulsive normal force acting on the ball during the impact

    M_imp = M(y1(end,1:n)'); 

    F_mtrx = [0 0; m_b -m_b];
    K = R*J_imp(1:2,:)*(M_imp\J_imp(1:2,:)'*(R'*F_mtrx))+[0 0;0 -1];
    Vr_bf = K\([Vr_si(1); e*(Vr_bi(2)-Vr_si(2))] - R*J_imp(1:2,:)*y(end,n+3:end-2)');
    Vr_bf(1) = Vr_bi(1);
%     Vr_bi(1)
%     Vr_bf(1)
%     Vr_bi(2)
%     Vr_bf(2)
%     Vr_bf(1) = (3*m_b*(Vr_bi(1)-Vr_si(1))/(2*m_b+3)) + Vr_si(1);  %%%%%%%%%%%%%% adding rotation to the ball will change this
%     Vr_bf(1) = Vr_bi(1); % post-impact tangential velocity of the ball
    
%     Ff = m_b*(Vr_bf(1) - Vr_bi(1)); % impulsive friction force acting on the ball during the impact
    Ff = 0; % No friction between surface and the ball
    Fr = F_mtrx*[Vr_bi(1); Vr_bf(2)];
    % The impulsive forces acting on the arm during impact are -N and -Ff.
    % We need to rotate the coordinate system back so we can get Fx_ee and
    % Fy_ee and the final ball velocities
    V_bf = R'*Vr_bf;
%     V_bf(V_bf<1e-8)=0;
    F = R'*Fr;
    
    
    % using the Eqn 13 from "Impact  Configurations  and  Measures for Kinematically  Redundant  and Multiple  Armed Robot Systems", Ian D. Walker
    
    delta_dq = M_imp\J_imp(1:2,:)'*F;
   
    q = [y(end,1:n+2)'; y(end,n+3:end-2)'+delta_dq; V_bf(1); V_bf(2)];
  
    [t1,y1]= ode45(@(t1,Q)n_link_ball_dynamics_FL_PD(Q,M,C,Tg,B,g),[t(end) t(end)+4],q,opts);
    
    if abs(y(end,n+2)) < .005 % if you hit the ground break
        break
    end
    
end
% bball_1_dof_animate(t,y,r,d)





n_link_ball_animate(t,y,L,r,d)