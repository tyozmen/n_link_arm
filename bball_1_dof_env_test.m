%clear all


env = bball_1_dof_Env();
%plot(env);

reset(env);
n_episodes = 3000;
begin = tic;
for i=1:n_episodes
    step(env,0);
%     if mod(i,5) == 0
%     end
end

fprintf("EPS: %f \n",  n_episodes/toc(begin));


[states, actions, t] = get_arrays(env);
bball_1_dof_animate(t,y,env.r,env.d)