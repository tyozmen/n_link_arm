n = 3;
[M,C,Tg,B] = manipEqns(n);
[M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
env = n_link_ball_Env(M_eq,C_eq,Tg_eq,B, ones(n,1), false);
env.N = 2500;

nDelta = 32;
nTop = 2;
nEpochs = 2000;
stepSize = 0.2;
deltaStd = 0.005;
nSeeds = 8;

R = zeros(nSeeds,1);
A = cell(nSeeds,1);


for i = 1:nSeeds
    agent = ARSAgent(env, stepSize, deltaStd, nDelta, nTop, useBias=true, maxStepsPerEpisode=env.N);
    rewards = agent.learn(nEpochs, verbose=1);

    [r,x] = doArsRollout(agent.policy, agent.env, env.N);
    R(i) = r;
    A{i} = agent;
    sprintf("seed: %i,  Reward: %f", i, r)
end

figure()
plot(rewards)
%% 
[r,X] = doArsRollout(agent.policy, agent.env, env.N);
[states, actions, t] = get_arrays(agent.env);
r

[states, actions, t] = get_arrays(env);
n_link_ball_animate(env.t_arr,env.states_arr',env.L,env.r,env.d)

figure()
plot(X);
hold on;
%plot(repmat(env.target_state', size(X,1)), '--')
%plot(xhist(1,:), xhist(3,:));


figure()
plot(actions')
r
