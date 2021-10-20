clear all
close all

env = bball_1_dof_Env_apex_control();
%plot(env);

reset(env);
n_episodes = 1000;
begin = tic;
y_des = [env.y_des];
h_apx = [env.h_apx];
offset = env.y_imp+env.d+env.r;
h_d_apx = [(env.h_d_apx+offset)];
R = 0;
for i=1:n_episodes
    [s,r,d,~] = env.step(0);
    y_des = [y_des env.y_des];
    h_apx = [h_apx env.h_apx];
    h_d_apx = [h_d_apx env.h_d_apx+offset];
    R = R + r;
%     if i> 650 
%         env.h_d_apx =  5;
%         env.A = .35;
%     end
%     if i > 1350
%         env.h_d_apx =  3;
%     end
%     if i > 2400
%         env.h_d_apx =  0.5-.1;
%         env.A = .05;
%     end
%     if mod(i,5) == 0
%     end
end

fprintf("FPS: %f \n",  n_episodes/toc(begin));

[states, actions, t] = get_arrays(env);
figure()
plot(t,states(1,:),t,y_des);
title('link position');
xlabel('t (s)'); ylabel('y (m)');
legend('y-actual)', 'y-desired)');
figure();
plot(t,h_apx,t,h_d_apx); %hold on; yline(env.h_d_apx+env.y_imp+env.d,'.-r')
legend('real apex','desired apex')
title('real apex height based on post impact velocity vs desired apex height');
figure();
plot(t,states(2,:));
title('ball height over time')
xlabel('t (s)'); ylabel('y (m)');
figure()
plot(t,[0 actions]);
R

bball_1_dof_animate(t,states',env.r,env.d) % uncomment to animate
