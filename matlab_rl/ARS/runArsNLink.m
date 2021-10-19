n = 5;
[M,C,Tg,B] = manipEqns(n);
[M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
env = n_link_arm_Env(M_eq,C_eq,Tg_eq,B);

nDelta = 32;
nTop = 32;
stepSize = .2;
deltaStd = .05;
nEpochs = 300;

begin = tic;
agent = ARSAgent(env, stepSize, deltaStd, nDelta, nTop, useBias=true);
rewards = agent.learn(nEpochs, verbose=1);
plot(rewards)
figure()
fprintf("EPS: %f \n",  nEpochs*2*nDelta/toc(begin));
%% 
[R,X] = doArsRollout(agent.policy, agent.env);
[states, actions, t] = get_arrays(agent.env);
L = ones(n,1)
nlink_animate(t,states',L)
figure()
plot(X);
hold on;
plot(repmat(env.target_state', size(X,1)), '--')
%plot(xhist(1,:), xhist(3,:));