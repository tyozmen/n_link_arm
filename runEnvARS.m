n = 2;
[M,C,Tg,B] = manipEqns(n);
[M_test,C_test,Tg_test] = sym2anonFnc(M,C,Tg);
env = n_link_arm_Env_Fast(M_test,C_test,Tg_test,B);

nDelta = 36;
nTop = 36;
alpha = .02;
sigma = .05;
N = 150;

begin = tic;
[W, policy] = EnvARSMu(env, alpha, sigma, nDelta, nTop, N);
fprintf("EPS: %f \n",  N*nDelta/toc(begin));

%%
% yvals = [-2:.05:0.5];
% xvals = [0:.05:10];
% [X,Y] = meshgrid(xvals,yvals);
% U = zeros(size(X));
% 
% for i = 1:size(X,1)
%     for j = 1:size(X,2)
%         U(i,j) = policy([X(i,j); Y(i,j)]);
%     end
% end
% 
% figure()
% U(U < -5) = -5;
% U(U > 5) = 5;
% surf(X,Y,U);
%policy = @(x)(W'*((x - policy_mean)./policy_std));
[R,xhist,thist] = DoRolloutWithEnv(policy,env); % n_delta*2 parrallel
%plot(xhist(1,:), xhist(3,:));