n = 2;
[M,C,Tg,B] = manipEqns(n);
[M_eq,C_eq,Tg_eq] = sym2anonFnc(M,C,Tg);
env = n_link_arm_Env(M_eq,C_eq,Tg_eq,B);

nDelta = 32;
nTop = 32;
alpha = .02;
sigma = .05;
N = 1000;

begin = tic;
[W, policy] = EnvARSMu(env, alpha, sigma, nDelta, nTop, N);
fprintf("EPS: %f \n",  N*2*nDelta/toc(begin));
%% 
[R,xhist,thist] = DoRolloutWithEnv(policy,env); % n_delta*2 parrallel
[states, actions, t] = get_arrays(env);
L = ones(n,1)
nlink_animate(t,states',L)
%plot(xhist(1,:), xhist(3,:));