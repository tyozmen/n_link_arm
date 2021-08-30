function [R,X,T] = DoRolloutWithEnv(policy,env)
% Does a rollout for ars given a policy function from observation->actions,
% and an environment object that implements Matlabs RL API

x = env.reset();
X = [];
isDone = false;
R = 0;

while(~isDone)
   a = policy(x);
   [x,r,isDone,~] = env.step(a);
   X = [X, x];
   R = R + r;
end

T = size(X,2);

end
