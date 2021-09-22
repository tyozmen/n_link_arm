function [W, policy] = EnvARSMu(env, alpha, sigma, nDelta, nTop, N)

W = zeros(env.getObservationInfo.Dimension(1), env.getActionInfo.Dimension(1) + 1);


%policy_mean = zeros(size(W,1),1);  % -- 10*1
policy_std = ones(size(W,1),1);    % -- 10*1
totalT = 0;

var_thresh = 1e-6;

for e = 1:N
    deltas = randn(size(W,1), size(W,2), nDelta)*sigma;    % -- 10*3*n_delta
    Wr = cat(3, W+deltas, W-deltas);    % -- 10*3*(2*n_delta)
    
    
    Rs = zeros(1, size(Wr,3));  % -- 1*(2*n_delta)
    %Xmus = zeros(size(W,1), size(Wr,3));   % -- 10*(2*n_delta)
    Xstds = ones(size(W,1), size(Wr,3));   % -- 10*(2*n_delta)
    rolloutTs = zeros(1, size(Wr,3));   % -- 1*(2*n_delta)
    
    begin = tic;
    
    parfor pii = 1:size(Wr,3)
        Wc = Wr(:,1:end-1,pii);   % -- 10*3
        Muc = Wr(:,end,pii);
        policy = @(x)(Wc'*((x - Muc)./policy_std)); % -- 3*1
        [R,X,T] = DoRolloutWithEnv(policy,env); % n_delta*2 parrallel rollouts
        Rs(pii) = R;
        %Xmus(:,pii) = mean(X,2);
        Xstds(:,pii) = std(X,0,2);
        rolloutTs(pii) = T;
    end
    
    
%     % update mean and std ---------------------------------------------------------
%     for mi = 1:size(Wr,3)
%         policy_mean = (policy_mean*totalT + rolloutTs(mi)*Xmus(:,mi))./(totalT + rolloutTs(mi));    % -- 10*1
%     end 
    
    for si = 1:size(Wr,3)
        if any(Xstds(:,si) < var_thresh)
           continue 
        end
        cur_var = policy_std.^2;    % -- 10*1
        new_var = Xstds(:,si).^2;  % -- 10*1
        
        updated_var = (cur_var*totalT + new_var*rolloutTs(si))./(totalT + rolloutTs(si));   % -- 10*1
        policy_std = sqrt(updated_var); % -- 10*1
    end
    % update mean and std ---------------------------------------------------------

   %policy_mean
   %policy_std
    
    totalT = totalT + sum(rolloutTs);
    if mod(e, 10) == 0
        fprintf("iteration %d, FPS: %f \n", e, sum(rolloutTs)/toc(begin));
        fprintf("max(sR): %f, mean(sR): %f \n\n", max(sR), mean(sR));
    end
    
    pR = Rs(1:nDelta); % -- 1*n_delta
    mR = Rs(nDelta+1:end); % -- 1*n_delta
    
    tR = max(pR, mR);   % -- 1*n_delta
    
    [sR, sI] = sort(tR,'descend'); %record, max(sR), mean(sR)
    Rdiff = (pR - mR);
    ss = alpha/(size(deltas,1)*std(Rs) + 1e-6);
    
    
    sortedRdiff = Rdiff(sI(1:nTop));
    sortedDeltas = deltas(:,:,sI(1:nTop)); 
    step = zeros(size(Wr,1), size(Wr,2)); 
    
    for j = 1:nTop
       step = step + sortedRdiff(j).*sortedDeltas(:,:,j);
    end
       
    W = W + ss*step;
 
    
    
end

policy = @(x)(W(:,1:end-1)'*((x - W(:,end))./policy_std)); % -- 3*1
%fprintf('done!');
%save('RL_50000iter_2maxtT_8alive.mat', 'W', 'policy_mean', 'policy_std', 'totaltime', 'a', 'sigma', 'N', 'n_delta', 'nTop', 'var_thresh', 'n', 'dt', 'maxT')
end