
for i = 1:10
agent = agent_arr(i);
n_steps = 2500;
agent.env.N = n_steps;
[R,X] = doArsRollout(agent.policy, agent.env, n_steps);

R
plot(X);
hold on
plot(ones(n_steps,1)*5, '--');
figure()
end