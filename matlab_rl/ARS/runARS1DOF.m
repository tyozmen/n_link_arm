env = bball_1_dof_Env_apex_control(false);

nDelta = 32;
nTop = 1;
stepSize = .2;
deltaStd = .1;
nEpochs = 200;

begin = tic;
agent = ARSAgent(env, stepSize, deltaStd, nDelta, nTop, useBias=false, maxStepsPerEpisode=1000);
rewards = agent.learn(nEpochs, verbose=1);


fprintf("EPS: %f \n",  nEpochs*2*nDelta/toc(begin));
plot(rewards);
figure()

%% 

[R,X] = doArsRollout(agent.policy, agent.env);
R 
[states, actions, t] = get_arrays(agent.env);
bball_1_dof_animate(t,states',env.r,env.d)
figure()
plot(X);
legend('paddle y', 'ball y', 'paddle dy', 'ball dy');