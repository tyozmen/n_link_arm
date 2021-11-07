load('ars_1dof_h4.mat')
%load('ars_works_7m.mat')

env = agent.env;

[r,X] = doArsRollout(agent.policy, env, env.N);
[states, actions, t] = get_arrays(agent.env);
r

[states, actions, t] = get_arrays(env);
bball_1_dof_animate(env.t_arr,env.states_arr',env.r,env.d)

figure()
plot(X);
c
figure()
plot(actions')