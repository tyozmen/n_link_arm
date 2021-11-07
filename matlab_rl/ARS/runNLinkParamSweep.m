n = 3;
[M,C,Tg,B] = manipEqns(n);
[M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
env = n_link_ball_Env(M_eq,C_eq,Tg_eq,B, ones(n,1), false);

nDelta = 32;
nTop = 2;
nEpochs = 1000;
stepSizes = [0.02, 0.2, 0.5];
deltaStds = [0.005, 0.02, 0.2];

nStep = size(stepSizes,2);
nStd = size(deltaStds,2);

R = zeros(nStep,nStd);
A = cell(nStep , nStd);

for i = 1:nStep
    for j = 1:nStd
        stepSize = stepSizes(i);
        deltaStd = deltaStds(j);
    
        begin = tic;
        agent = ARSAgent(env, stepSize, deltaStd, nDelta, nTop, useBias=true, maxStepsPerEpisode=env.N);
        rewards = agent.learn(nEpochs, verbose=1);

        [r,x] = doArsRollout(agent.policy, agent.env, env.N);
        R(i,j) = r;
        A{i,j} = agent;
    end
end

figure()
plot(rewards)
fprintf("EPS: %f \n",  nEpochs*2*nDelta/toc(begin));
%% 
[r,X] = doArsRollout(agent.policy, agent.env, env.N);
[states, actions, t] = get_arrays(agent.env);
r

[states, actions, t] = get_arrays(env);
n_link_ball_animate(env.t_arr,env.states_arr',env.L,env.r,env.d)

figure()
plot(X);
hold on;
plot(repmat(env.target_state', size(X,1)), '--')
plot(xhist(1,:), xhist(3,:));


figure()
plot(actions')
r