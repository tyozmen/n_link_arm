clear all
n = 4;
L = 0.5*ones(n,1);
[M,C,Tg,B] = manipEqns(n)
[M_test,C_test,Tg_test] = sym2anonFnc(M,C,Tg);
env = n_link_arm_Env_no_animation(M_test,C_test,Tg_test,B);

%env = n_link_arm_Env_Fast(M_test,C_test,Tg_test,B);
%plot(env);
reset(env);
begin = tic;
for i=1:3000
    step(env,zeros(env.n,1));
%     if mod(i,5) == 0
%     end
end
fprintf("EPS: %f \n",  500/toc(begin));
[states, actions, t] = get_arrays(env);

nlink_animate(t,states',L)
