%load('ars_nlink_apex_4_reward.mat')

%load('ars_nlink_many_seeds.mat')
%agent = A{1};

env = agent.env

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