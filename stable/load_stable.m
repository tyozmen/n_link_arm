modelDict = pyrunfile("load_stable.py", "model_dict");
algo = 'ppo';
seed = 1;

env = bball_1_dof_Env_apex_control(false);
obs = env.reset();

isDone=false;
rewards = []; 
while ~isDone
    ret = modelDict{algo}{seed}.predict(obs);
    act = double(ret{1});
    act = double(act);
    [obs,rew,isDone,~] = env.step(act);
    rewards = [rewards, rew];
end