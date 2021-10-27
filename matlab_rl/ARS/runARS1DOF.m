clear all
close all

R_arr = [];
R_tuned_arr = [];
agent_arr = [];
maxSteps = 2500;
for i = 1:10

    env = bball_1_dof_Env_apex_control(false);
    env.N = maxSteps;
    
    nDelta = 32;
    nTop = 1;
    stepSize = .2;
    deltaStd = .02;
    nEpochs = 500;
    
    begin = tic;
    agent = ARSAgent(env, stepSize, deltaStd, nDelta, nTop, useBias=true, maxStepsPerEpisode=maxSteps);
    rewards = agent.learn(nEpochs, verbose=1);
    
    
    %fprintf("EPS: %f \n",  nEpochs*2*nDelta/toc(begin));
    %plot(rewards)
    %figure()
    
    
    %% 
    
    %agent.policy = @(x)randsample([-1,1],1);
    %agent.policy = @(x)0;
    
    [R,X] = doArsRollout(agent.policy, agent.env, maxSteps);
    %plot(R);
    %[states, actions, t] = get_arrays(agent.env);
    %bball_1_dof_animate(t,states',env.r,env.d)
    %figure()
    %plot(X);
    %legend('paddle y', 'ball y', 'paddle dy', 'ball dy');
    R
    R_arr = [R_arr, R];

    
    %% 
    agent.stepSize = .01;
    agent.deltaStd = .005;
    nEpochs = 500;
    rewards = agent.learn(nEpochs, verbose=1);
    
    %% 
    
    [R,X] = doArsRollout(agent.policy, agent.env, maxSteps);
    %plot(R);
    [states, actions, t] = get_arrays(agent.env);
    %bball_1_dof_animate(t,states',env.r,env.d)
    %figure()
    %plot(X);
    %legend('paddle y', 'ball y', 'paddle dy', 'ball dy');
    %R
    R_tunned_arr = [R_tuned_arr, R];
    agent_arr = [agent_arr agent];
end