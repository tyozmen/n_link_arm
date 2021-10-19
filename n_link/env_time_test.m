%clear all
n = 3;
L = 1*ones(n,1);
[M,C,Tg,B] = manipEqns(n);
[M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);

env = n_link_ball_Env(M_eq,C_eq,Tg_eq,B,L,true);
%plot(env);

reset(env);
n_episodes = 3000;
begin = tic;
for i=1:n_episodes
    step(env,zeros(env.n,1));
%     if mod(i,5) == 0
%     end
end

fprintf("EPS: %f \n",  n_episodes/toc(begin));


%[states, actions, t] = get_arrays(env);
%nlink_animate(t,states',L)
