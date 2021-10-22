py.sys.setdlopenflags(int32(10));
[modelDict, normDict] = pyrunfile("load_stable.py", ["model_dict", "norm_dict"]);
algo = 'ppo';
seed = 7;

env = bball_1_dof_Env_apex_control(false);
obs = env.reset();

isDone=false;
rewards = []; 
while ~isDone
    normObs = normDict{algo}{seed}.normalize_obs(obs);
    ret = modelDict{algo}{seed}.predict(normObs);
    act = double(ret{1});
    act = double(act);
    [obs,rew,isDone,~] = env.step(act);
    rewards = [rewards, rew];
end


sum(rewards)
[states, actions, t] = env.get_arrays();

plot(states')
legend("y_s", "y_b", "dy_s", "dy_b")
figure()
plot(actions')
