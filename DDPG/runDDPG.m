n = 5;
[M,C,Tg,B] = manipEqns(n);
[M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
env = n_link_arm_Env(M_eq,C_eq,Tg_eq,B);
agent = rlDDPGAgent(env.getObservationInfo,env.getActionInfo);
opt = rlTrainingOptions('MaxEpisodes',1000,"UseParallel",true,"MaxStepsPerEpisode",400);
trainstats = train(agent, env, opt);
%% 
agent.generatePolicyFunction()
p = @(x)evaluatePolicy(x)'
[R,xhist,thist] = DoRolloutWithEnv(p,env); % n_delta*2 parrallel
[states, actions, t] = get_arrays(env);
L = ones(n,1)
nlink_animate(t,states',L)
