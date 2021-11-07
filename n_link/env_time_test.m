clear all; close all;
n = 3;
L = 1*ones(n,1);
[M,C,Tg,B] = manipEqns(n);
[M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);

env = n_link_ball_Env(M_eq,C_eq,Tg_eq,B,L,true);
%plot(env);
h_d_apx = [];
h_apx = [];
reset(env);
n_steps = 15000;
begin = tic;
for i=1:n_steps
    step(env,zeros(env.n,1));
    h_d_apx = [h_d_apx env.h_d_apx];
    h_apx = [h_apx env.h_apx];
%     if mod(i,5) == 0
%     end
end

fprintf("EPS: %f \n",  n_steps/toc(begin));


[states, actions, t] = get_arrays(env);
% n_link_ball_animate(env.t_arr,env.states_arr',env.L,env.r,env.d)
figure(); plot(env.t_arr,env.states_arr(1:3,:)); hold on; plot(env.t_arr(1:end-1),env.q_d_arr);legend('1','2','3','d1','d2','d3')
figure(); plot(env.t_arr,env.states_arr(4:5,:)); 
figure(); plot(env.t_arr(1:end-1),h_d_apx,env.t_arr(1:end-1),h_apx);
figure(); plot(env.t_arr(1:end-1),env.actions_arr);legend('1','2','3')