clear all
close all


env = bball_1_dof_Env_apex_control();
%plot(env);

reset(env);
n_steps = 1000;
begin = tic;
y_des = [env.y_des];
h_apx = [env.h_apx];
offset = env.y_imp+env.d+env.r;
h_d_apx = [(env.h_d_apx+offset)];

R = 0
for i=1:n_steps
    [s,r,d,~] = step(env,0);
    y_des = [y_des env.y_des];
    h_apx = [h_apx env.h_apx];
    h_d_apx = [h_d_apx env.h_d_apx+offset];
    R = R + r;
%     if mod(i,5) == 0
%     end
end

fprintf("Model Based Controller ------------------------------\n");
fprintf("FPS: %f \n",  n_steps/toc(begin));
fprintf("total err^2 reward: %f \n",  R);


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

% bball_1_dof_animate(t,states',env.r,env.d) % uncomment to animate


%% -----------------------------------------------------------


env = bball_1_dof_Env_apex_control(false);
load('ars_1dof_agent.mat');
%plot(env);

s = reset(env);
n_steps = 1000;
begin = tic;
y_des = [env.y_des];
h_apx = [env.h_apx];
offset = env.y_imp+env.d+env.r;
h_d_apx = [(env.h_d_apx+offset)];
R = 0
for i=1:n_steps
    act = agent.policy(s);
    [s,r,d,~] = step(env,act);
    y_des = [y_des env.y_des];
    h_apx = [h_apx env.h_apx+env.y_imp];
    R = R + r;
%     if mod(i,5) == 0
%     end
end


fprintf("ARS Controller ------------------------------\n");
fprintf("FPS: %f \n",  n_steps/toc(begin));
fprintf("total err^2 reward: %f \n",  R);

[states, actions, t] = get_arrays(env);
figure()
plot(t,states(1,:),t,y_des);
title('link position ARS');
xlabel('t (s)'); ylabel('y (m)');
legend('y-actual)', 'y-desired)');
figure();
plot(t,h_apx,t,h_d_apx); %hold on; yline(env.h_d_apx+env.y_imp+env.d,'.-r')
legend('real apex','desired apex')
title('real apex height based on post impact velocity vs desired apex height');
figure();
plot(t,states(2,:));
title('ball height over time ARS')
xlabel('t (s)'); ylabel('y (m)');

% bball_1_dof_animate(t,states',env.r,env.d) % uncomment to animate