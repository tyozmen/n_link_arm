clear all
close all

env = bball_1_dof_Env_apex_control();
%plot(env);

reset(env);
n_episodes = 1000;
begin = tic;
y_des = [env.y_des];
h_apx = [env.h_apx+env.y_imp];
R = 0
for i=1:n_episodes
    [s,r,d,~] = step(env,0);
    y_des = [y_des env.y_des];
    h_apx = [h_apx env.h_apx+env.y_imp];
    R = R + r;
%     if mod(i,5) == 0
%     end
end

fprintf("FPS: %f \n",  n_episodes/toc(begin));

[states, actions, t] = get_arrays(env);

plot(t,states(1,:),t,y_des);
title('link position');
xlabel('t (s)'); ylabel('y (m)');
legend('y-actual)', 'y-desired)');
figure();
plot(h_apx); hold on; yline(env.h_d_apx+env.y_imp,'.-r')
title('calculated apex height values based on states vs desired apex height');
figure();
plot(t,states(2,:));
title('ball height over time')
xlabel('t (s)'); ylabel('y (m)');

bball_1_dof_animate(t,states',env.r,env.d) % uncomment to animate
figure()