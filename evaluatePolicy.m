function action1 = evaluatePolicy(observation1)
%#codegen

% Reinforcement Learning Toolbox
% Generated on: 22-Sep-2021 17:31:14

numAction = 5;
% Get the actions mean and standard deviation
meanAndStd = localEvaluate(observation1);
actionMean = meanAndStd(1:numAction);
actionStd = meanAndStd(numAction+1:end);
% Sample action from mean and standard deviation
action1 = actionMean + actionStd .* randn(size(actionMean),'like',actionMean);
end
%% Local Functions
function meanAndStd = localEvaluate(observation1)
persistent policy
if isempty(policy)
	policy = coder.loadDeepLearningNetwork('agentData.mat','policy');
end
observation1 = observation1(:)';
meanAndStd = predict(policy, observation1);
end