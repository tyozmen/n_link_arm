env = bball_1_dof_Env_apex_control(false);

nDelta = 32;
nTop = 4;
stepSize = .02;
deltaStd = .01;
nEpochs = 1000;

begin = tic;
agent = ARSAgent(env, stepSize, deltaStd, nDelta, nTop, useBias=false, maxStepsPerEpisode=1000);
%rewards = agent.learn(nEpochs, verbose=1);


fprintf("EPS: %f \n",  nEpochs*2*nDelta/toc(begin));
plot(rewards)
figure()

%% 

%agent.policy = @(x)randsample([-1,1],1);
%agent.policy = @(x)0;

[R,X] = doArsRollout(agent.policy, agent.env, 1000);
R; 
[states, actions, t] = get_arrays(agent.env);
bball_1_dof_animate(t,states',env.r,env.d)
figure()
plot(X);
legend('paddle y', 'ball y', 'paddle dy', 'ball dy');