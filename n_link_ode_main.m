n = 3;
[M_sym,C_sym,Tg_sym,B] = manipEqns(n);
[M,C,Tg] = sym2anonFnc(M_sym,C_sym,Tg_sym); %get M,C,Tg as anonymous functions
g = 9.81;
%% generate variable string to pass through anon functions
q_vars = [];
dq_vars = [];
for i = 1:n
    if i == n
        q_vars = [q_vars sprintf('Q(%d)',i)];
        dq_vars = [dq_vars sprintf('Q(%d)',i+n)];
    else

        q_vars = [q_vars sprintf('Q(%d),',i)];
        dq_vars = [dq_vars sprintf('Q(%d),',i+n)];
    end 
end

full_vrs = [q_vars ',' dq_vars];
%% Run ODE CAUTION!!!!!!!!! Change initial conditions for ODE if 'n' is changed 
% No control input. Arm should settle pointing down due to friction
opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30);
[t,X]= ode45(@(t,Q)n_link_dynamics(Q,M,C,Tg,B,g,q_vars,full_vrs),[0 15],[pi/3 pi/2 pi-.3 0 0 0],opts);
 
nlink_animate(t,X,ones(n,1))

% Feedback Linearization to cancel the effects of gravity and friction. Arm
% should be able stay at initial conditions.
opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30);
[t_FL,X_FL]= ode45(@(t_FL,Q)n_link_dynamics_FeedbackLinearization(Q,M,C,Tg,B,g,q_vars,full_vrs),[0 3],[pi/3 pi/2 pi-.3 0 0 0],opts);
 
nlink_animate(t_FL,X_opt,ones(n,1))





