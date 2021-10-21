env = bball_1_dof_Env_apex_control(false);
agent = rlPPOAgent(env.getObservationInfo,env.getActionInfo);
opt = rlTrainingOptions('MaxEpisodes',2000,"UseParallel",false,"MaxStepsPerEpisode",499);
trainstats = train(agent, env, opt);

%% 
p = @(x)double(cell2mat(agent.getAction({x})));
[R,X] = doArsRollout(p,env); % n_delta*2 parrallel
[states, actions, t] = get_arrays(env);
bball_1_dof_animate(t,states',env.r,env.d)
