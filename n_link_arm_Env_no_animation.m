classdef n_link_arm_Env_no_animation < rl.env.MATLABEnvironment
    %fast environment for the n-link arm. Uses anonymous functions for M,C,Tg instead of symbolics    
    
    %% Properties 
    properties
        
        g = 9.8 % The constant acceleration in y. 
        M = [];
        C = [];
        Tg = [];
        B = [];
        n = 3; % number of links

        dt = .01; 

        N = 1000; % how many steps i n an episode %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lim = 1000; % bound for torques
        ptch=[]
        init_qvals = [];
        Figure = [];
        q_vrs = [];
        full_vrs = [];
        % Initialize system state [x,dx,y,dy]'
        X = [];
        curStep = 0;
        t = [];
        states_arr = [];
        actions_arr = [];
        t_arr = [];
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = n_link_arm_Env_no_animation(M,C,Tg,B)
            n = length(B); %number of links
           
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([2*n 1]);
            ObservationInfo.Name = 'n_link system states';
            ObservationInfo.Description = sprintf('q1-q%d, dq1-dq%d',n,n);
            
            % Initialize Action settings   
            lim = 100;
            ActionInfo = rlNumericSpec([n 1], 'LowerLimit', -lim*ones(n,1), 'UpperLimit', lim*ones(n,1));
            ActionInfo.Name = 'n_link Action';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            this.lim = lim;
            this.init_qvals = [0:5*this.dt:pi];
            this.n = n;
            this.X = zeros(2*n,1);
            this.M = M;
            this.C = C;
            this.Tg = Tg;
            this.B = B;
            
            this.states_arr = [];
            this.actions_arr = [];
            this.t_arr = [];
            this.t = 0;
            % Create a string of states to pass through maniplator Eqn
            % matrices
            q_vars = [];
            dq_vars = [];
            for i = 1:n
                if i == n
                    q_vars = [q_vars sprintf('this.X(%d)',i)];
                    dq_vars = [dq_vars sprintf('this.X(%d)',i+n)];
                else

                    q_vars = [q_vars sprintf('this.X(%d),',i)];
                    dq_vars = [dq_vars sprintf('this.X(%d),',i+n)];
                end 
            end
            this.q_vrs = q_vars;
            this.full_vrs = [q_vars ',' dq_vars];
           
           
            
            
            
            % Initialize property values and pre-compute necessary values
            % this.ActionInfo.Elements = this.MaxForce*[-1 1];

        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
            Action = max(-this.lim, Action);
            Action = min(this.lim, Action);
            
            q = this.X(1:this.n,1);
            dq = this.X(this.n+1:2*this.n,1);
            
            % pass the string of variables(states) through the anonymous
            % functions (Manip Eqn matrices)
            eval(sprintf(['Tg_tmp = this.Tg(' this.q_vrs ');']));
            eval(sprintf(['M_tmp = this.M(' this.q_vrs ');']));
            eval(sprintf(['C_tmp = this.C(' this.full_vrs ');']));
           
            d2q = double(-M_tmp\C_tmp*dq-M_tmp\Tg_tmp*this.g+M_tmp\this.B*Action);
            
            q = q + dq*this.dt;
            dq = dq + d2q*this.dt;
 
            this.X(1:this.n,1) = q;
            this.X(this.n+1:2*this.n,1) = dq;
            Observation = this.X;
            this.t = this.t + this.dt;
            
            this.states_arr = [this.states_arr Observation];
            this.actions_arr = [this.actions_arr Action];
            this.t_arr = [this.t_arr this.t];
            % 0 for now will be changed to reflect a desired position
            Reward = -sum((0-this.X).^2);



           
            IsDone = this.curStep >= this.N; %|| term;
            this.curStep = this.curStep + 1;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            
            % notifyEnvUpdated(this);
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
            this.X(1:this.n,1) = randsample(this.init_qvals,this.n);
            this.X(this.n+1:end,1) = 0;
            this.states_arr = [];
            this.actions_arr = [];
            this.t_arr = [];
            this.t = 0;
            
            InitialObservation = this.X;
            this.states_arr = [this.states_arr InitialObservation];
            
            this.t_arr = [this.t_arr 0];
%             this.ptch = [];
            this.curStep = 0; 
            
            % notifyEnvUpdated(this);
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods   
        
        
        % Helper methods to create the environment
        % Discrete force 1 or 2
%         function force = getForce(this,action)
%             if ~ismember(action,this.ActionInfo.Elements)
%                 error('Action must be %g for going left and %g for going right.',-this.MaxForce,this.MaxForce);
%             end
%             force = action;           
%         end
%         % update the action info based on max force
%         function updateActionInfo(this)
%             this.ActionInfo.Elements = this.MaxForce*[-1 1];
%         end
%         
%         % Reward function
%         function Reward = getReward(this)
%             if ~this.IsDone
%                 Reward = this.RewardForNotFalling;
%             else
%                 Reward = this.PenaltyForFalling;
%             end          
%         end
%         
%         % (optional) Visualization method
        function plot(this)
            this.Figure = figure('Color','w','Renderer','zbuffer');
            ha = gca(this.Figure);
            ha.XLimMode = 'manual';
            ha.YLimMode = 'manual';
            ha.XLim = [-5 5];
            ha.YLim = [-5 5];

            % Update the visualization
            envUpdatedCallback(this)
        end
        
        function [obs_arr, u_arr, t_arr] = get_arrays(this)
            obs_arr = this.states_arr;
            u_arr = this.actions_arr;
            t_arr = this.t_arr;
        end

    end
    
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        
        function envUpdatedCallback(this)
            if ~isempty(this.Figure) && isvalid(this.Figure)
            q = this.X(1:this.n,1);
            L = ones(this.n,1);
            p = this.ptch;
            
            if ~isempty(p)
                for j=1:this.n
                    set(p(j), 'Visible', 'off') 
                end
            end
            for i = 1:this.n
            

                th = .06; % half-THickness of arm

                avals = pi*[0:.05:1];
                x(i,:) = [0 L(i) L(i)+th*cos(avals-pi/2) L(i) 0 th*cos(avals+pi/2)];
                y(i,:) = [-th -th th*sin(avals-pi/2) th th th*sin(avals+pi/2)];
                r(i,:) = (x(i,:).^2 + y(i,:).^2).^.5;
                a(i,:) = atan2(y(i,:),x(i,:));
                if i ==1
                    xdraw(:,i) = r(i,:).*sin(a(i,:)+q(i));  % x pts to plot, for Link i
                    ydraw(:,i) = r(i,:).*cos(a(i,:)+q(i));  % y pts to plot, for Link i
                    xend(i) = L(i)*sin(q(i));  % "elbow" at end of Link i, x
                    yend(i) = L(i)*cos(q(i));  % "elbow" at end of Link i, x
                else
                    xdraw(:,i) = xend(:,i-1) + r(i,:).*sin(a(i,:)+q(i));  % x pts to plot, for Link i
                    ydraw(:,i) = yend(:,i-1) + r(i,:).*cos(a(i,:)+q(i));  % y pts to plot, for Link i
                    xend(i) = xend(i-1) + L(i)*sin(q(i));  % "elbow" at end of Link i, x
                    yend(i) = yend(i-1) + L(i)*cos(q(i));
                end
                
            
                this.Figure; 
                p(i) = patch(xdraw(:,i),ydraw(:,i),'b','FaceAlpha',.3); hold on %pole i
            end
            this.ptch = p;
            drawnow();
            end
        end
    end
end
